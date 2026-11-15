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

"""Deterministic midterm scheduler with atomic reservations and serial entry."""

from dataclasses import dataclass, replace
import hashlib
import math

from .planning import allocate
from .scheduling import entry_conflict, footprint, polygon_gap
from .verification import DwellVerifier


@dataclass(frozen=True)
class Command:
    """A side effect for the ROS adapter; tokens isolate attempts and callbacks."""

    kind: str
    robot_id: str
    token: str
    slot: int


class Engine:
    """Run ordinary and cooperative tasks through the same execution path."""

    def __init__(self, config, run_id, completed=()):
        self.config, self.run_id = config, run_id
        self.status = 'WAITING'
        self.completed = set(completed)
        self.index = 0
        self.attempts = {}
        self.failures = {}
        self.recovery_required = False
        self.assignment = ()
        self.slots = {}
        self.commands = []
        self.events = []
        self.verifier = DwellVerifier()
        self.started_at = None
        self.releasing_at = None
        self.failure = ''
        self.retry = False
        self.last_now = None
        self.task_id = ''
        self.distances = {}
        self.dispatch_gate = ''
        self.holding_since = {}
        self.confirmed_parked = set()
        self.retry_avoid = {}
        self.retry_not_before = 0.0

    @property
    def target(self):
        return self.config.targets[self.index] if self.index < len(self.config.targets) else None

    def emit(self, kind, now, reason=''):
        self.events.append({'type': kind, 'sim_time': now, 'reason': reason,
                            'target_id': self.target.target_id if self.target else '',
                            'task_id': self.task_id, 'robot_ids': list(self.assignment),
                            'dwell_elapsed_s': self.verifier.elapsed})

    def drain_commands(self):
        commands, self.commands = self.commands, []
        return commands

    def accepted(self, token, accepted, now):
        """Late acceptances remain owned until cancelled, never start a new attempt."""
        if token not in self.slots:
            return
        slot = self.slots[token]
        if slot['state'] != 'sending':
            return
        slot['state'] = 'active' if accepted else 'ended'
        if self.status in ('RELEASING', 'HALTED', 'RECOVERING'):
            if accepted:
                self.commands.append(Command('cancel', slot['robot'], token, slot['index']))
        elif not accepted:
            self.fail('ACTION_REJECTED', now, retry=True)

    def transit_accepted(self, token, accepted, now):
        """Reserve the robot on approach acceptance without verifying a rescue."""
        self.accepted(token, accepted, now)
        if accepted and token in self.slots and self.slots[token]['state'] == 'active':
            self.slots[token]['state'] = 'transit'

    def transit_finished(self, token):
        """Wait for the state stream to confirm idle before sending rescue."""
        if token in self.slots and self.status == 'ACTIVE':
            self.slots[token]['state'] = 'transit_ready'

    def rescue_requested(self, token, now):
        """Keep the reservation across approach and final rescue actions."""
        self.slots[token]['state'], self.slots[token]['sent_at'] = 'sending', now

    def ended(self, token, now, reason='ACTION_ENDED'):
        """Unexpected terminal results fail the attempt; intentional release does not."""
        if token not in self.slots:
            return
        self.slots[token]['state'] = 'ended'
        if self.status == 'ACTIVE':
            self.fail(reason, now, retry=True)

    def release(self, now):
        self.status = 'RELEASING'
        self.releasing_at = now
        self.verifier.reset()
        for token, slot in self.slots.items():
            if slot['state'] == 'queued':
                slot['state'] = 'ended'
            elif slot['state'] in ('active', 'transit'):
                self.commands.append(Command('cancel', slot['robot'], token, slot['index']))
            elif slot['state'] == 'transit_ready':
                slot['state'] = 'ended'

    def fail(self, reason, now, retry=False):
        if self.status != 'ACTIVE':
            return
        if self.verifier.start is not None:
            self.emit('DWELL_RESET', now, reason)
        self.failure, self.retry = reason, retry
        if retry:
            tid = self.target.target_id
            self.failures[tid] = self.failures.get(tid, 0) + 1
            if len(self.assignment) == 1:
                self.retry_avoid[tid] = list(self.assignment)
            self.retry_not_before = now + self.config.settings.retry_backoff_s
        self.emit('FAILED', now, reason)
        self.release(now)

    def interrupt(self, reason, now):
        """Cancel immediately; only the adapter may authorize safe re-entry."""
        if self.status == 'ACTIVE':
            self.recovery_required = True
            self.fail(reason, now)
        elif self.status == 'RELEASING':
            if self.target and self.target.target_id not in self.completed:
                if self.retry and not self.recovery_required:
                    tid = self.target.target_id
                    self.failures[tid] = max(0, self.failures.get(tid, 0) - 1)
                    self.retry_avoid.pop(tid, None)
                self.recovery_required = True
            self.retry = False
            self.failure = self.failure or reason

    def resume(self, now):
        """Resume only after fresh phase, physics, idle and orphan-goal checks."""
        if self.status != 'RECOVERING':
            return
        self.slots, self.assignment, self.commands = {}, (), []
        self.verifier = DwellVerifier()
        self.distances = {}
        self.recovery_required = False
        self.failure = ''
        self.last_now = now
        self.retry_not_before = now
        self.status = 'WAITING'
        self.holding_since.clear()
        self.confirmed_parked.clear()
        self.emit('RECOVERED', now)

    def _parked(self, token, slot, robots):
        robot = robots.get(slot['robot'])
        return (slot['state'] == 'active' and robot and robot.mode == 4
                and robot.active_task == self.task_id and robot.stationary()
                and math.dist((robot.x, robot.y), self.target.slots[slot['index']][:2])
                <= self.config.settings.holding_position_tolerance_m)

    def can_enter(self, index, robots):
        """Admit one mover only after every earlier reservation is stably parked."""
        earlier = [(t, s) for t, s in self.slots.items() if s['index'] < index]
        for token, slot in earlier:
            robot = robots[slot['robot']]
            since = self.holding_since.get(token)
            if (not self._parked(token, slot, robots) or since is None
                    or robot.stamp - since < self.config.settings.holding_stable_s):
                self.dispatch_gate = 'WAITING_FOR_STABLE_PARKING:' + robot.robot_id
                return False
        slot = next(s for s in self.slots.values() if s['index'] == index)
        robot = robots[slot['robot']]
        peers = [r for rid, r in robots.items() if rid in self.config.robots
                 and rid != robot.robot_id]
        conflict = entry_conflict(self.target, index, robot, peers, self.config.settings)
        if conflict:
            self.dispatch_gate = 'ENTRY_OCCUPIED:' + conflict
            return False
        self.dispatch_gate = ''
        return True

    def _send_next(self, now, robots):
        ordered = sorted(self.slots.items(), key=lambda pair: pair[1]['index'])
        for token, slot in ordered:
            if slot['state'] == 'queued':
                if not self.can_enter(slot['index'], robots):
                    return
                slot['state'], slot['sent_at'] = 'sending', now
                self.commands.append(Command('send', slot['robot'], token, slot['index']))
                self.emit('NAVIGATION_STARTED', now)
                return

    def tick(self, now, robots, enabled, path_cost):
        """Advance without blocking; all durations here use ROS simulation time."""
        if self.last_now is not None and now < self.last_now:
            self.interrupt('CLOCK_REWIND', now)
        self.last_now = now
        if not enabled:
            self.interrupt('PHASE_OR_SAFETY_STOP', now)
        if self.status == 'RELEASING':
            ended = all(s['state'] == 'ended' for s in self.slots.values())
            idle = all(r in robots and robots[r].idle(now, self.config.settings.state_stale_s)
                       for r in self.assignment)
            if ended and idle:
                self.emit('RELEASED', now)
                if self.target.target_id in self.completed:
                    self.index += 1
                    self.status = 'WAITING'
                    self.recovery_required = False
                elif self.recovery_required:
                    self.status = 'RECOVERING'
                elif self.retry and self.failures.get(self.target.target_id, 0) < \
                        self.config.settings.max_attempts and enabled:
                    self.status = 'WAITING'
                else:
                    self.status = 'HALTED'
                self.assignment, self.slots = (), {}
            elif now - self.releasing_at > self.config.settings.release_timeout_s:
                self.failure = 'RELEASE_NOT_CONFIRMED'
                self.status = 'RECOVERING' if self.recovery_required else 'HALTED'
                self.emit('HALTED', now, self.failure)
            return
        if self.status == 'WAITING':
            while self.target and self.target.target_id in self.completed:
                self.index += 1
            if self.target is None:
                self.status = 'DONE'
                return
            if not enabled:
                return
            if now < self.retry_not_before:
                self.dispatch_gate = 'RETRY_BACKOFF'
                return
            if not all(r in robots and robots[r].idle(now, self.config.settings.state_stale_s)
                       for r in self.config.robots):
                self.dispatch_gate = 'WAITING_FOR_IDLE_FLEET'
                return
            available = {r: robots[r] for r in self.config.robots
                         if r in robots and robots[r].idle(now, self.config.settings.state_stale_s)}
            assignment = allocate(available, self.target.slots, path_cost, self.target.approaches,
                                  self.config.settings.arrival_tolerance_m,
                                  self.retry_avoid.get(self.target.target_id, ()))
            if assignment is None:
                self.dispatch_gate = 'NO_REACHABLE_ASSIGNMENT'
                return
            self.assignment = assignment
            attempt = self.attempts.get(self.target.target_id, 0) + 1
            self.attempts[self.target.target_id] = attempt
            prefix = hashlib.sha256(self.run_id.encode()).hexdigest()[:12]
            self.task_id = f'rescue-{prefix}-{self.target.target_id}-{attempt}'
            self.slots = {f'{self.task_id}:{i}': {'robot': r, 'index': i, 'state': 'queued'}
                          for i, r in enumerate(assignment)}
            self.started_at, self.failure, self.retry = now, '', False
            self.verifier = DwellVerifier()
            self.holding_since.clear()
            self.confirmed_parked.clear()
            self.status = 'ACTIVE'
            self.emit('ASSIGNED', now)
        if self.status != 'ACTIVE' or not enabled:
            return
        settings = self.config.settings
        for rid in self.config.robots:
            if rid not in robots or not robots[rid].healthy(now, settings.state_stale_s):
                self.interrupt('FLEET_STATE_INVALID', now)
                return
            owned = any(s['robot'] == rid and s['state'] != 'queued'
                        for s in self.slots.values())
            if not owned and not robots[rid].idle(now, settings.state_stale_s):
                self.interrupt('UNRESERVED_ROBOT_MOVING', now)
                return
        fleet = [robots[r] for r in self.config.robots]
        if polygon_gap(*(footprint(r.x, r.y, r.yaw, settings) for r in fleet)) \
                <= settings.entry_clearance_m:
            self.interrupt('FLEET_CLEARANCE_LOST', now)
            return
        for token, slot in self.slots.items():
            robot = robots.get(slot['robot'])
            if robot is None or not robot.healthy(now, settings.state_stale_s):
                self.fail('ROBOT_STATE_INVALID', now, retry=True)
                return
            if robot.active_task and robot.active_task != self.task_id:
                self.fail('ROBOT_OWNERSHIP_CHANGED', now)
                return
            if slot['state'] == 'sending' and now - slot['sent_at'] > \
                    settings.action_response_timeout_s:
                self.fail('ACTION_RESPONSE_TIMEOUT', now, retry=True)
                return
            if (slot['state'] in ('active', 'transit') and robot.mode != 4
                    and now - slot['sent_at'] > settings.navigation_timeout_s):
                self.fail('NAVIGATION_TIMEOUT', now, retry=True)
                return
            if self._parked(token, slot, robots):
                self.holding_since.setdefault(token, robot.stamp)
                self.confirmed_parked.add(token)
            else:
                self.holding_since.pop(token, None)
                if token in self.confirmed_parked:
                    self.interrupt('PARKING_LOST', now)
                    return
        if now - self.started_at > settings.partner_wait_timeout_s:
            self.fail('TASK_TIMEOUT', now, retry=True)
            return
        self._send_next(now, robots)
        selected = [robots[r] for r in self.assignment]
        owned = (all(s['state'] == 'active' for s in self.slots.values())
                 and all(r.active_task == self.task_id for r in selected))
        # Start only after the robots have stopped at the navigation slots. This is an
        # engineering admission policy, not an extra official scoring rule.
        admitted = owned and (self.verifier.start is not None or all(r.mode == 4 for r in selected))
        before = self.verifier.start
        guarded_target = replace(self.target, dwell=self.target.dwell + settings.completion_guard_s)
        complete, reset, self.distances = self.verifier.update(
            now, selected, guarded_target, settings, valid=admitted)
        if reset:
            self.emit('DWELL_RESET', now, 'OUTSIDE_OR_INVALID_SAMPLE')
        if self.verifier.start is not None and (before is None or reset):
            self.emit('DWELL_STARTED', now)
        if complete:
            self.completed.add(self.target.target_id)
            self.emit('COMPLETED', now)
            self.release(now)

    def snapshot(self):
        """Return a JSON-safe view with explicit target-centred geometry."""
        target = self.target
        task_state = self.status
        if self.status == 'ACTIVE':
            task_state = ('DWELLING' if self.verifier.start is not None
                          else 'WAITING_FOR_PARTNER' if any(
                              s['state'] == 'queued' for s in self.slots.values())
                          else 'NAVIGATING')
        return {'status': self.status, 'task_state': task_state, 'task_id': self.task_id,
                'target_id': target.target_id if target else '',
                'center': list(target.center) if target else None,
                'radius_m': target.radius if target else None,
                'required_dwell_s': target.dwell if target else None,
                'dwell_elapsed_s': self.verifier.elapsed,
                'completed_targets': sorted(self.completed), 'failure': self.failure,
                'assignment': list(self.assignment), 'attempts': dict(self.attempts),
                'failures': dict(self.failures), 'recovery_required': self.recovery_required,
                'dispatch_gate': self.dispatch_gate, 'retry_avoid': dict(self.retry_avoid),
                'retry_not_before': self.retry_not_before,
                'distances_m': dict(self.distances)}
