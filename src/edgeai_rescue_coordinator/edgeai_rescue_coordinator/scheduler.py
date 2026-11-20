# Copyright 2026 EdgeAI Robotics Challenge Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Deterministic dependency scheduling, driven exclusively by external events."""

from collections import deque
from copy import deepcopy
from enum import IntEnum


class StepState(IntEnum):
    """Values from MissionStepState.msg."""

    WAITING_FOR_PREDECESSORS = 0
    READY = 1
    IN_PROGRESS = 2
    COMPLETED = 3
    FAILED = 4
    BLOCKED = 5
    CANCELLED = 6


class MissionState(IntEnum):
    """Values from MissionExecutionState.msg."""

    IDLE = 0
    EXECUTING = 1
    COMPLETED = 2
    FAILED = 3
    ABORTED = 4


class Reason(IntEnum):
    """Values from TaskReason.msg; no scheduler-specific wire codes."""

    NONE = 0
    USER_REQUEST = 1
    PHASE_CHANGED = 2
    RESET_REQUESTED = 3
    RELEASED_BY_COORDINATOR = 4
    NAVIGATION_FAILED = 100
    NAVIGATION_TIMEOUT = 101
    LOCALIZATION_LOST = 102
    HOLD_TIMEOUT = 103
    RELOCALIZATION_FAILED = 104
    ROBOT_UNAVAILABLE = 200
    PARTNER_UNAVAILABLE = 201
    ZONE_EXITED = 300
    DWELL_RESET = 301
    DEPENDENCY_FAILED = 400
    TARGET_INVALID = 401
    PLAN_INVALID = 500
    SAFETY_STOP = 600
    INTERNAL_ERROR = 900
    CONFIGURATION_INVALID = 901


_TERMINAL = {StepState.COMPLETED, StepState.FAILED,
             StepState.BLOCKED, StepState.CANCELLED}
_MISSION_TERMINAL = {MissionState.COMPLETED, MissionState.FAILED, MissionState.ABORTED}
_ZERO = {'sec': 0, 'nanosec': 0}


def _reason(code=Reason.NONE, detail=''):
    return {'code': int(code), 'detail': detail}


def _time(value):
    if not isinstance(value, dict) or set(value) != {'sec', 'nanosec'}:
        raise ValueError('time must have sec and nanosec')
    sec, nano = value['sec'], value['nanosec']
    if type(sec) is not int or not -2**31 <= sec < 2**31:
        raise ValueError('invalid time sec')
    if type(nano) is not int or not 0 <= nano < 10**9:
        raise ValueError('invalid time nanosec')
    return sec, nano


def _plan_order(plan):
    # Scheduling preconditions only; target/context validation belongs upstream.
    if not isinstance(plan, dict) or plan.get('valid') is not True:
        raise ValueError('PLAN_INVALID: expected an already validated plan')
    for key in ('run_id', 'mission_id'):
        if not isinstance(plan.get(key), str) or not plan[key]:
            raise ValueError('PLAN_INVALID: missing identity')
    steps = plan.get('steps')
    if not isinstance(steps, list) or not steps:
        raise ValueError('PLAN_INVALID: steps must be nonempty')
    by_id = {}
    for step in steps:
        if not isinstance(step, dict):
            raise ValueError('PLAN_INVALID: invalid step')
        for key in ('step_id', 'target_id'):
            if not isinstance(step.get(key), str) or not step[key]:
                raise ValueError('PLAN_INVALID: missing step identity')
        sid = step['step_id']
        if sid in by_id:
            raise ValueError('PLAN_INVALID: duplicate step_id')
        priority = step.get('priority')
        if type(priority) is not int or not 0 <= priority < 2**32:
            raise ValueError('PLAN_INVALID: invalid priority')
        preds = step.get('predecessor_step_ids')
        if not isinstance(preds, list) or any(not isinstance(x, str) for x in preds):
            raise ValueError('PLAN_INVALID: invalid predecessors')
        if len(set(preds)) != len(preds):
            raise ValueError('PLAN_INVALID: duplicate predecessor')
        by_id[sid] = step
    children = {sid: [] for sid in by_id}
    degrees = {}
    for sid, step in by_id.items():
        degrees[sid] = len(step['predecessor_step_ids'])
        for pred in step['predecessor_step_ids']:
            if pred not in children:
                raise ValueError('PLAN_INVALID: unknown predecessor')
            children[pred].append(sid)
    queue = deque(sid for sid in by_id if degrees[sid] == 0)
    order = []
    while queue:
        sid = queue.popleft()
        order.append(sid)
        for child in children[sid]:
            degrees[child] -= 1
            if degrees[child] == 0:
                queue.append(child)
    if len(order) != len(steps):
        raise ValueError('PLAN_INVALID: dependency cycle')
    return order


class DependencyScheduler:
    """Own one mission's step states, without assigning robots or deciding rescue success."""

    def __init__(self, plan):
        """Copy a validated MissionPlan dictionary and initialize dependency readiness."""
        self._order = _plan_order(plan)
        self._plan = deepcopy(plan)
        self._definitions = {s['step_id']: s for s in self._plan['steps']}
        header = self._plan.get('header')
        if not isinstance(header, dict) or set(header) != {'stamp', 'frame_id'}:
            raise ValueError('PLAN_INVALID: invalid header')
        _time(header['stamp'])
        if not isinstance(header['frame_id'], str):
            raise ValueError('PLAN_INVALID: invalid frame_id')
        self._header = deepcopy(header)
        self._state = MissionState.IDLE
        self._reason = _reason()
        self._last = dict.fromkeys(self._order)
        self._steps = {
            sid: {'step_id': sid, 'target_id': step['target_id'],
                  'state': int(StepState.WAITING_FOR_PREDECESSORS), 'task_id': '',
                  'robot_ids': [], 'completed_at': dict(_ZERO), 'reason': _reason()}
            for sid, step in self._definitions.items()}
        self._refresh()

    def _refresh(self):
        for sid in self._order:
            step = self._steps[sid]
            if step['state'] not in (StepState.WAITING_FOR_PREDECESSORS, StepState.READY):
                continue
            preds = [self._steps[p]['state']
                     for p in self._definitions[sid]['predecessor_step_ids']]
            if any(s in (StepState.FAILED, StepState.BLOCKED, StepState.CANCELLED)
                   for s in preds):
                step.update(state=int(StepState.BLOCKED),
                            reason=_reason(Reason.DEPENDENCY_FAILED))
            elif all(s == StepState.COMPLETED for s in preds):
                step['state'] = int(StepState.READY)
        states = [step['state'] for step in self._steps.values()]
        if all(s == StepState.COMPLETED for s in states):
            self._state, self._reason = MissionState.COMPLETED, _reason()
        elif all(s in _TERMINAL for s in states):
            self._state = MissionState.FAILED
            # First unsuccessful step in plan order, not hash or event order.
            self._reason = next(deepcopy(s['reason']) for s in self._steps.values()
                                if s['state'] != StepState.COMPLETED)

    def _stamp(self, stamp):
        if _time(stamp) > _time(self._header['stamp']):
            self._header['stamp'] = deepcopy(stamp)

    def apply(self, event):
        """Apply an external event; return False for malformed, stale or illegal events."""
        if self._state in _MISSION_TERMINAL or not isinstance(event, dict):
            return False
        if any(event.get(k) != self._plan[k] for k in ('run_id', 'mission_id')):
            return False
        try:
            stamp = event['stamp']
            when = _time(stamp)
        except (KeyError, ValueError):
            return False
        kind = event.get('kind')
        if kind in ('phase_changed', 'safety_stop'):
            if when < _time(self._header['stamp']):
                return False
            code = Reason.PHASE_CHANGED if kind == 'phase_changed' else Reason.SAFETY_STOP
            for step in self._steps.values():
                if step['state'] not in _TERMINAL:
                    step.update(state=int(StepState.CANCELLED), reason=_reason(code))
            self._state, self._reason = MissionState.ABORTED, _reason(code)
            self._stamp(stamp)
            return True
        sid = event.get('step_id')
        if not isinstance(sid, str) or sid not in self._steps:
            return False
        step = self._steps[sid]
        if step['state'] in _TERMINAL:
            return False
        if self._last[sid] is not None and when < self._last[sid]:
            return False
        task, robots = event.get('task_id'), event.get('robot_ids')
        if not isinstance(task, str) or not task:
            return False
        if (not isinstance(robots, list) or not robots
                or any(not isinstance(r, str) or not r for r in robots)
                or len(set(robots)) != len(robots)):
            return False
        reason = event.get('reason', _reason(
            Reason.INTERNAL_ERROR if kind == 'fail' else
            Reason.USER_REQUEST if kind == 'cancel' else Reason.NONE))
        if (not isinstance(reason, dict) or set(reason) != {'code', 'detail'}
                or type(reason['code']) is not int
                or reason['code'] not in set(Reason)
                or not isinstance(reason['detail'], str)):
            return False
        state = step['state']
        if kind == 'start' and state == StepState.READY:
            if any(s['task_id'] == task for s in self._steps.values()):
                return False
            step.update(state=int(StepState.IN_PROGRESS), task_id=task,
                        robot_ids=list(robots), reason=_reason())
        elif kind in ('complete', 'fail', 'cancel') and state == StepState.IN_PROGRESS:
            if task != step['task_id'] or robots != step['robot_ids']:
                return False
            target = {'complete': StepState.COMPLETED, 'fail': StepState.FAILED,
                      'cancel': StepState.CANCELLED}[kind]
            step.update(state=int(target), reason=deepcopy(reason))
            if kind == 'complete':
                step.update(completed_at=deepcopy(stamp), reason=_reason())
            elif reason['code'] == Reason.NONE:
                step['reason'] = _reason(Reason.INTERNAL_ERROR if kind == 'fail'
                                         else Reason.USER_REQUEST)
        else:
            return False
        self._last[sid] = when
        self._stamp(stamp)
        self._state = MissionState.EXECUTING
        self._refresh()
        return True

    def snapshot(self):
        """Return a detached snapshot matching MissionExecutionState.msg field for field."""
        return deepcopy({'header': self._header, 'run_id': self._plan['run_id'],
                         'mission_id': self._plan['mission_id'], 'state': int(self._state),
                         'steps': list(self._steps.values()), 'reason': self._reason})

    def ready_steps(self):
        """Return detached step definitions ordered by descending priority, then step_id."""
        ready = [s for sid, s in self._definitions.items()
                 if self._steps[sid]['state'] == StepState.READY]
        return deepcopy(sorted(ready, key=lambda s: (-s['priority'], s['step_id'])))
