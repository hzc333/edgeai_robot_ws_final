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

"""Execute frozen-target rescues for both stages with one allocator and verifier."""

from copy import deepcopy
from dataclasses import dataclass, field
from itertools import combinations, permutations
import json
import math
from pathlib import Path
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from edgeai_common.qos import qos_from_contract
from edgeai_rescue_coordinator.ros_support import message, to_dict
from edgeai_rescue_coordinator.scheduler import DependencyScheduler
from edgeai_robot_interfaces.action import ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState, MissionExecutionState, MissionPlan, MissionStepState, RescueEvent,
    RescueTask, RobotState, TaskReason,
)
from edgeai_robot_interfaces.srv import SetCompetitionPhase
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .evidence import Journal
from .frozen_inputs import context, load_targets, Rules
from .model import Target
from .scheduling import entry_conflict, footprint, polygon_gap
from .verification import DwellVerifier


# Engineering limits for Task 2A retries; not organizer-prescribed numbers.
# Navigation failures are mostly another robot or a passing obstruction in the way, so they
# get a few extra attempts after a pause in which the blocker can move on.
NAVIGATION_RETRIES = 3
RETRY_BACKOFF_S = 5.0


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def completion_key(event):
    return event['scoring_stage'], event['mission_id'], event['step_id'] or event['target_id']


@dataclass
class Goal:
    """Keep the assigned action UUID until its terminal result is reconciled."""

    uuid: list
    probe: object = None
    probe_since: float = 0.0
    unknown_at: float = 0.0


@dataclass
class Assignment:
    """One rescue attempt with ordered robots, slots and its own dwell clock."""

    task: RescueTask
    target: Target
    deadline: float
    goals: dict = field(default_factory=dict)
    verifier: DwellVerifier = field(default_factory=DwellVerifier)
    ending: bool = False
    retry_cancel_at: float = 0.0
    # Robots not yet sent, as (zone index, robot ID); they enter one after another.
    queue: list = field(default_factory=list)
    parked_since: dict = field(default_factory=dict)


class MissionExecution:
    """Adapt frozen targets and dependency steps to the same rescue action path."""

    def __init__(self, coordinator, contract, target_path, parameters_path, step_timeout,
                 inject_outcomes=False):
        if not math.isfinite(step_timeout) or step_timeout <= 0:
            raise ValueError('rescue_step_timeout_s must be finite and positive')
        self.node = coordinator
        self.step_timeout = step_timeout
        self.retry_backoff = coordinator.declare_parameter(
            'rescue_retry_backoff_s', RETRY_BACKOFF_S).value
        if not math.isfinite(self.retry_backoff) or self.retry_backoff < 0:
            raise ValueError('rescue_retry_backoff_s must be finite and not negative')
        self.target_path = Path(target_path)
        self.parameters_path = parameters_path
        self.test_mode = coordinator.get_parameter('test_mode').value
        self.inject_outcomes = inject_outcomes
        self.rules = None
        self.context = None
        self.targets = {}
        self.journal = None
        self.scheduler = None
        self.last_state = None
        self.pending = None
        self.seen = set()
        self.assigned = {}
        self.orphans = {}
        self.next_orphan_poll = 0.0
        self.basic_failed = set()
        self.basic_attempts = {}
        self.navigation_retries = {}
        self.retry_at = {}
        self.retry_avoid = {}
        self.finish = None
        self.last_status = None
        self.last_grid = None
        self.valid_parking = {}
        topics = contract['topics']
        snapshot = coordinator.qos['snapshot']
        self.state_pub = coordinator.create_publisher(
            MissionExecutionState, topics['mission_state'], snapshot)
        self.status_pub = coordinator.create_publisher(String, '/edge/rescue/status', snapshot)
        assignment_qos = qos_from_contract(contract, 'snapshot')
        assignment_qos.depth = len(coordinator.config.robots)
        self.tasks_pub = coordinator.create_publisher(
            RescueTask, topics['rescue_tasks'], assignment_qos)
        coordinator.create_subscription(MissionPlan, topics['mission_plan'], self.on_plan, snapshot)
        if inject_outcomes:
            coordinator.create_subscription(RescueEvent, topics['rescue_events'],
                                            self.on_rescue_event, coordinator.qos['event_stream'])

    @property
    def has_goals(self):
        return bool(self.assigned or self.orphans)

    def prime(self, now):
        """Bind to the immutable Task 1 snapshot once its associated phase is ready."""
        phase = self.node.phase
        if (self.journal or now <= 0 or phase is None
                or phase.phase not in (phase.PHASE_TASK2A, phase.PHASE_TASK2B)
                or not phase.run_id or not self.target_path.is_file()):
            return
        node = self.node
        self.context = context(phase)
        path = self.parameters_path
        config = node.get_parameter('config_dir').value or None
        self.rules = Rules.load(path, phase, config, test_mode=self.test_mode)
        self.targets, digest = load_targets(
            self.target_path, self.context, self.rules.geometry_version,
            official=phase.run_mode == phase.RUN_MODE_COMPETITION)
        self.journal = Journal(node.get_parameter('output_dir').value, {
            'run_id': phase.run_id, 'map_id': phase.map_id, 'map_revision': phase.map_revision,
            'target_revision': phase.target_revision, 'targets_sha256': digest,
            'parameters_sha256': phase.parameters_sha256,
            'config_sha256': node.config_hash,
        }, completion_key=completion_key)
        if self.journal.monitor_started is None:
            self.journal.event({'type': 'MONITOR_STARTED', 'sim_time': now})
        node.collision.monitor_started = self.journal.monitor_started
        previous = self.journal.previous
        if previous:
            self._restore(previous, phase)
        node.get_logger().info(f'Frozen rescue evidence: {self.journal.directory}')

    def _restore(self, previous, phase):
        self.node.collision.restore(previous['collision'])
        self.seen.update(previous['seen'])
        self.basic_attempts = previous['basic_attempts']
        self.basic_failed.update(previous['basic_failed'])
        self.navigation_retries = previous.get('navigation_retries', {})
        self.orphans = {rid: Goal(value) for rid, value in previous['goals'].items()}
        self.last_state = previous['mission_state']
        if self.last_state and self.last_state['state'] in (
                MissionExecutionState.STATE_IDLE, MissionExecutionState.STATE_EXECUTING):
            # Preserve terminal steps and credit durable completion receipts. A crashed
            # mission is not re-dispatched from a cached transient-local plan.
            state = self.last_state
            reason = {'code': TaskReason.SAFETY_STOP, 'detail': 'allocator restarted'}
            for step in state['steps']:
                receipt = (RescueTask.STAGE_TASK2B, state['mission_id'], step['step_id'])
                if (receipt in self.journal.completed
                        and step['state'] == MissionStepState.STATE_IN_PROGRESS):
                    step.update(state=MissionStepState.STATE_COMPLETED,
                                reason={'code': TaskReason.NONE, 'detail': ''})
                elif step['state'] in (MissionStepState.STATE_WAITING_FOR_PREDECESSORS,
                                       MissionStepState.STATE_READY,
                                       MissionStepState.STATE_IN_PROGRESS):
                    step.update(state=MissionStepState.STATE_CANCELLED, reason=reason)
            state.update(state=MissionExecutionState.STATE_ABORTED, reason=reason)
        if self.last_state:
            self.state_pub.publish(message(MissionExecutionState, deepcopy(self.last_state)))
        self._save()

    def close(self):
        """Cancel active goals and release the run's evidence lock."""
        if self.journal:
            self.abort()
            self._release_goals()
            self.journal.close()

    def on_plan(self, plan):
        """Take a validated plan without overwriting an active mission or replaying an ID."""
        if not plan.valid or not plan.mission_id or plan.mission_id in self.seen:
            return
        if self.node.phase and self.node.phase.phase != CompetitionState.PHASE_TASK2B:
            return
        if self.context is not None and context(plan) != self.context:
            return
        if self.pending is not None or self._active():
            self.node.get_logger().warning(f'Skipped {plan.mission_id}: a mission is active')
            return
        self.pending = plan

    def _active(self):
        return self.scheduler is not None and self.last_state['state'] in (
            MissionExecutionState.STATE_IDLE, MissionExecutionState.STATE_EXECUTING)

    def _save(self):
        if self.journal is None:
            return
        goals = {rid: goal.uuid for assignment in self.assigned.values()
                 for rid, goal in assignment.goals.items()}
        goals.update({rid: goal.uuid for rid, goal in self.orphans.items()})
        self.journal.atomic('snapshot.json', {
            'seen': sorted(self.seen), 'goals': goals, 'mission_state': self.last_state,
            'collision': self.node.collision.history(), 'basic_attempts': self.basic_attempts,
            'basic_failed': sorted(self.basic_failed),
            'navigation_retries': self.navigation_retries,
        })

    def _publish(self):
        self.last_state = self.scheduler.snapshot()
        self._save()
        self.state_pub.publish(message(MissionExecutionState, deepcopy(self.last_state)))

    def _apply(self, kind, assignment=None, reason=None):
        previous = self.last_state['header']['stamp']
        stamp = to_dict(self.node.get_clock().now().to_msg())
        if (stamp['sec'], stamp['nanosec']) < (previous['sec'], previous['nanosec']):
            stamp = previous
        event = {'kind': kind, 'stamp': stamp, 'run_id': self.last_state['run_id'],
                 'mission_id': self.last_state['mission_id']}
        if assignment is not None:
            task = assignment.task
            event.update(step_id=task.step_id, task_id=task.task_id,
                         robot_ids=list(task.assigned_robot_ids))
        if reason is not None:
            event['reason'] = to_dict(reason)
        if not self.scheduler.apply(event):
            raise RuntimeError(f'Dependency scheduler rejected {kind} for {event}')
        self._publish()

    def _interrupt(self):
        node = self.node
        phase = node.phase
        if (node.shutdown_requested or node.estop or node.last_error or node.clock_rewound
                or (phase and (phase.estop_active or phase.phase == phase.PHASE_ESTOP))):
            return 'safety_stop'
        if (phase is None or phase.phase not in (phase.PHASE_TASK2A, phase.PHASE_TASK2B)
                or phase.frozen or phase.reset_in_progress
                or (self.context is not None and context(phase) != self.context)):
            return 'phase_changed'
        if (time.monotonic() - node.phase_received > node.config.settings.phase_heartbeat_timeout_s
                or node.collision.count or node.collision.error):
            return 'safety_stop'
        if node.contact_feed and node.collision.snapshot(
                node._now(), time.monotonic())['state'] != 'CLEAR':
            return 'safety_stop'
        if self.assigned and any(
                a.task.scoring_stage != self._stage() and not a.ending
                for a in self.assigned.values()):
            return 'phase_changed'
        if self._active() and self._stage() != RescueTask.STAGE_TASK2B:
            return 'phase_changed'
        return None

    def _stage(self):
        return (RescueTask.STAGE_TASK2A if self.node.phase.phase == CompetitionState.PHASE_TASK2A
                else RescueTask.STAGE_TASK2B)

    def abort(self, kind='safety_stop'):
        """Abort the mission and release actions; Task 2A may resume after a new phase."""
        self.pending = None
        changed = self._active()
        if self._active():
            self._apply(kind)
        code = TaskReason.SAFETY_STOP if kind == 'safety_stop' else TaskReason.PHASE_CHANGED
        for assignment in self.assigned.values():
            if not assignment.ending:
                assignment.ending = True
                changed = True
                self._event(assignment, RescueEvent.EVENT_CANCELLED, TaskReason(code=code))
        if changed:
            self._save()

    def fail_closed(self):
        """Attempt cancellation even if the evidence store caused the failure."""
        try:
            self.abort()
        finally:
            for assignment in self.assigned.values():
                assignment.ending = True
            self._release_goals()

    def tick(self, now):
        """Advance supervision, dwell, timeouts, release, and eligible dispatch."""
        if self.journal is None:
            if self.node.phase is not None and self.node._rescue_phase():
                self._status('WAITING_FOR_FROZEN_TARGETS')
            return
        self._record_contacts()
        interrupt = self._interrupt()
        if interrupt and (self.assigned or self._active()):
            self.abort(interrupt)
        self._release_goals()
        if interrupt or now <= 0:
            self._status('SAFETY_STOP' if interrupt == 'safety_stop' else 'WAITING_FOR_PHASE')
            return
        if self.orphans:
            self._status('RECOVERING_ACTIONS')
            return
        if self.node.grid is None and self.node.get_parameter('require_map').value:
            self._status('WAITING_FOR_MAP')
            return
        for assignment in list(self.assigned.values()):
            if assignment.ending:
                continue
            if time.monotonic() >= assignment.deadline:
                self._finish(assignment, 'fail', TaskReason.NAVIGATION_TIMEOUT,
                             'rescue step exceeded its execution timeout')
            else:
                self._send_next(assignment)
                if not self.inject_outcomes:
                    self._verify(assignment, now)
        if self._stage() == RescueTask.STAGE_TASK2A:
            self._dispatch_ready(self._basic_steps(), now)
            self._basic_status()
        else:
            if self.pending is not None and not self.assigned:
                self._start_plan()
            if self._active():
                self._dispatch_ready(self.scheduler.ready_steps(), now)
            self._status('TASK2B_ACTIVE' if self._active() else 'WAITING_FOR_MISSION')

    def _start_plan(self):
        plan, self.pending = self.pending, None
        if context(plan) != self.context or plan.mission_id in self.seen:
            return
        try:
            for step in plan.steps:
                self.targets[step.target_id]
            payload = to_dict(plan)
            payload['header']['stamp'] = to_dict(self.node.get_clock().now().to_msg())
            self.scheduler = DependencyScheduler(payload)
        except (KeyError, TypeError, ValueError) as error:
            self.node.get_logger().error(f'Unusable plan {plan.mission_id}: {error}')
            return
        self.seen.add(plan.mission_id)
        self._publish()

    def _basic_steps(self, include_paused=False):
        busy = {a.task.target_id for a in self.assigned.values()}
        wall = time.monotonic()
        return [{'step_id': '', 'target_id': tid, 'priority': 0, 'predecessor_step_ids': []}
                for tid, target in sorted(self.targets.items())
                if target['eligible'] and tid not in self.basic_failed and tid not in busy
                and (include_paused or self.retry_at.get(tid, 0.0) <= wall)
                and (RescueTask.STAGE_TASK2A, '', tid) not in self.journal.completed]

    def _basic_status(self):
        if self.assigned:
            self._status('TASK2A_ACTIVE')
        elif self._basic_steps():
            self._status('WAITING_FOR_REACHABLE_ASSIGNMENT')
        elif self._basic_steps(include_paused=True):
            self._status('TASK2A_RETRY_PAUSE')
        else:
            # Freeze also when some targets failed for good: Task 2B can still run, and the
            # status and evidence name the failed targets.
            failed = sorted(self.basic_failed)
            self._status('TASK2A_FAILED' if failed else 'TASK2A_DONE')
            node = self.node
            if (node.get_parameter('request_finish').value
                    and self.finish is None and node.phase_client.service_is_ready()):
                reason = ('all frozen Task 2A targets rescued' if not failed
                          else 'Task 2A attempts exhausted for ' + ', '.join(failed))
                self.finish = node.phase_client.call_async(SetCompetitionPhase.Request(
                    run_id=self.context[0], requested_phase=CompetitionState.PHASE_FROZEN,
                    requester_type=SetCompetitionPhase.Request.REQUESTER_SYSTEM,
                    requester_node=node.get_fully_qualified_name(),
                    reason=reason + ' and actions released'))

    def _status(self, status):
        completed = sorted(key[2] for key in self.journal.completed
                           if key[0] == RescueTask.STAGE_TASK2A) if self.journal else []
        payload = {'status': status, 'completed_targets': completed,
                   'failed_targets': sorted(self.basic_failed),
                   'active_targets': sorted(a.task.target_id for a in self.assigned.values()
                                            if not a.ending),
                   'error': self.node.last_error or self.node.collision.error}
        if payload != self.last_status:
            self.status_pub.publish(String(data=json.dumps(payload)))
            self.last_status = payload

    def _dispatch_ready(self, steps, now):
        node = self.node
        reserved = {rid for a in self.assigned.values() for rid in a.task.assigned_robot_ids}
        occupied_targets = {a.task.target_id for a in self.assigned.values()}
        reserved_slots = [slot for a in self.assigned.values() for slot in a.target.slots]
        available = {
            rid: robot for rid, robot in node.robots.items()
            if rid not in reserved and node.raw_states[rid].run_id == self.context[0]
            and robot.idle(now, node.config.settings.state_stale_s)
            and not node.raw_states[rid].faults
            and node.action_clients[(rid, 'rescue')].server_is_ready()
        }
        for step in steps:
            target_id = step['target_id']
            if target_id in occupied_targets:
                continue
            data = self.targets[target_id]
            slots = data['slots']
            count = len(slots)
            if len(available) < count:
                continue
            dwell = self.rules.normal_dwell if count == 1 else self.rules.cooperative_dwell
            target = Target(target_id, 0, slots[0][:2], self.rules.tolerance, count, dwell, slots)
            robots = self._choose(target, available, reserved_slots, now)
            if not robots:
                continue
            for rid in robots:
                del available[rid]
            occupied_targets.add(target_id)
            reserved_slots.extend(slots)
            self._start(step, robots, target, data['zones'])

    def _choose(self, target, available, reserved_slots, now):
        node = self.node
        settings = node.config.settings
        slots = target.slots
        if node.grid is not self.last_grid:
            self.valid_parking = {}
            self.last_grid = node.grid
        if node.grid is not None and target.target_id not in self.valid_parking:
            try:
                node.grid.validate_parking([target], settings)
            except ValueError as error:
                node.get_logger().warning(str(error))
                self.valid_parking[target.target_id] = False
            else:
                self.valid_parking[target.target_id] = True
        if node.grid is not None and not self.valid_parking[target.target_id]:
            return None
        parked = [footprint(*slot, settings) for slot in slots]
        margin = settings.entry_clearance_m + 2 * self.rules.arrival
        if (any(polygon_gap(a, b) <= margin for a, b in combinations(parked, 2))
                or any(polygon_gap(p, footprint(*slot, settings)) <= margin
                       for p in parked for slot in reserved_slots)):
            return None
        # Check actual peer footprints as well as reserved future slots. Nav2 owns routes.
        peers = [r for r in node.robots.values() if r.healthy(now, settings.state_stale_s)]
        cost = node.grid if node.grid is not None else math.dist
        # After a failure prefer other robots: the one that failed is often hemmed in (for
        # example next to its partner after a cooperative hold) and would fail the same way.
        # A robot already parked on one of this target's zones keeps that zone: moving it
        # sideways to the neighbouring zone is a tight manoeuvre and makes its partner squeeze
        # in from the other side.
        avoid = self.retry_avoid.get(target.target_id, ())
        tolerance = self.rules.arrival
        choices = []
        for group in combinations(sorted(available), target.required):
            for robots in permutations(group):
                distances = []
                for i, r in enumerate(robots):
                    here = (available[r].x, available[r].y)
                    distances.append(0.0 if math.dist(here, slots[i][:2]) <= tolerance
                                     else cost(here, slots[i][:2]))
                if not all(math.isfinite(d) for d in distances) or any(entry_conflict(
                        target, i, available[r], [p for p in peers if p.robot_id not in group],
                        settings) for i, r in enumerate(robots)):
                    continue
                displaced = sum(
                    any(j != i and math.dist((available[r].x, available[r].y), slots[j][:2])
                        <= tolerance for j in range(len(slots)))
                    for i, r in enumerate(robots))
                penalty = sum(r in avoid for r in robots)
                choices.append((penalty, displaced, max(distances), sum(distances), robots))
        return min(choices)[4] if choices else None

    def _start(self, step, robots, target, zones):
        node = self.node
        stamp = node.get_clock().now().to_msg()
        task2b = self._stage() == RescueTask.STAGE_TASK2B
        poses = [message(PoseStamped, {'header': {'frame_id': 'map', 'stamp': to_dict(stamp)},
                                       'pose': zone['pose']}) for zone in zones]
        task = RescueTask(
            run_id=self.context[0], map_id=self.context[1], map_revision=self.context[2],
            task_id=uuid.uuid4().hex, target_id=target.target_id, scoring_stage=self._stage(),
            mission_id=self.last_state['mission_id'] if task2b else '', step_id=step['step_id'],
            task_type=RescueTask.TYPE_NORMAL if len(robots) == 1 else RescueTask.TYPE_COOPERATIVE,
            state=RescueTask.STATE_ASSIGNED, priority=step['priority'],
            assigned_robot_ids=list(robots),
            goal_poses=poses, zone_tolerance_m=self.rules.tolerance)
        task.header.frame_id = 'map'
        task.header.stamp = stamp
        ns = round(target.dwell * 1e9)
        task.required_dwell.sec, task.required_dwell.nanosec = divmod(ns, 10**9)
        if task2b:
            predecessors = set(step['predecessor_step_ids'])
            task.predecessor_task_ids = [s['task_id'] for s in self.last_state['steps']
                                         if s['step_id'] in predecessors]
        assignment = Assignment(task, target, time.monotonic() + self.step_timeout)
        assignment.queue = list(enumerate(robots))
        self.assigned[task.task_id] = assignment
        if task2b:
            self._apply('start', assignment)
        else:
            self.basic_attempts[target.target_id] = self.basic_attempts.get(target.target_id, 0) + 1
            self._save()
        self._event(assignment, RescueEvent.EVENT_ASSIGNED)
        self.tasks_pub.publish(task)
        self._send_next(assignment)

    def _send_next(self, assignment):
        """Send one more robot once every robot already sent is stably parked.

        Robots sent together to a cooperative target block each other in doorways and the
        start area, so they enter one after another as in the midterm engine. The next one
        is the first queued robot whose zone no other robot occupies: a teammate still parked
        on that zone from an earlier rescue has to move to its own zone first.
        """
        if assignment.ending or not assignment.queue:
            return
        node = self.node
        settings = node.config.settings
        task = assignment.task
        for i, sent in enumerate(task.assigned_robot_ids):
            if sent not in assignment.goals:
                continue
            robot = node.robots.get(sent)
            if not (robot is not None and robot.mode == RobotState.MODE_HOLDING
                    and robot.active_task == task.task_id and robot.stationary()
                    and math.dist((robot.x, robot.y), assignment.target.slots[i][:2])
                    <= settings.holding_position_tolerance_m):
                assignment.parked_since.pop(sent, None)
                return
            since = assignment.parked_since.setdefault(sent, robot.stamp)
            if robot.stamp - since < settings.holding_stable_s:
                return
        # Teammates already parked on their own zones are where the plan puts them; _choose
        # checked those zones against each other. Their actual heading can differ from the
        # zone's nominal one, which would otherwise make the neighbouring zone look occupied.
        for index, rid in assignment.queue:
            robot = node.robots.get(rid)
            peers = [r for r in node.robots.values()
                     if r.robot_id != rid and r.robot_id not in assignment.goals]
            if robot is not None and not entry_conflict(
                    assignment.target, index, robot, peers, settings):
                break
        else:
            return
        assignment.queue.remove((index, rid))
        assignment.goals[rid] = Goal(list(uuid.uuid4().bytes))
        self._save()
        try:
            self._dispatch(assignment, index, rid)
        except Exception as error:
            self._finish(assignment, 'fail', TaskReason.INTERNAL_ERROR, str(error))

    def _dispatch(self, assignment, index, rid):
        task = assignment.task
        goal = ExecuteRescueTask.Goal(
            run_id=task.run_id, map_id=task.map_id, map_revision=task.map_revision,
            task_id=task.task_id, target_id=task.target_id, target_type=task.task_type,
            rescue_zone_index=index, rescue_pose=task.goal_poses[index],
            arrival_xy_tolerance_m=self.rules.arrival)
        client = self.node.action_clients[(rid, 'rescue')]
        future = client.send_goal_async(goal, goal_uuid=UUID(uuid=assignment.goals[rid].uuid))
        future.add_done_callback(
            lambda f, a=assignment, robot=rid: self._accepted(a, robot, f))

    def _accepted(self, assignment, robot, future):
        if self.journal is None:
            return
        try:
            handle = future.result()
            if handle.accepted:
                handle.get_result_async().add_done_callback(
                    lambda f: self._ended(assignment, robot, f))
            else:
                assignment.goals.pop(robot, None)
                self._save()
                self._finish(assignment, 'fail', TaskReason.NAVIGATION_FAILED,
                             f'{robot} rejected the rescue action')
        except Exception as error:
            self._finish(assignment, 'fail', TaskReason.INTERNAL_ERROR, str(error))

    def _ended(self, assignment, robot, future):
        if self.journal is None:
            return
        try:
            result = future.result().result
        except Exception as error:
            self._finish(assignment, 'fail', TaskReason.INTERNAL_ERROR, str(error))
            return
        assignment.goals.pop(robot, None)
        self._save()
        code = result.reason.code or TaskReason.NAVIGATION_FAILED
        self._finish(assignment, 'fail', code, robot=robot, detail=(
            f'{robot} action ended before verifier completion: {result.reason.detail}'))

    def _release_goals(self):
        wall = time.monotonic()
        if self.orphans and wall >= self.next_orphan_poll:
            self.next_orphan_poll = wall + 0.5
            if self._poll_goals(self.orphans, wall):
                self._save()
        for key, assignment in list(self.assigned.items()):
            if assignment.ending and wall >= assignment.retry_cancel_at:
                assignment.retry_cancel_at = wall + 0.5
                changed = self._poll_goals(assignment.goals, wall)
                if not assignment.goals:
                    del self.assigned[key]
                if changed:
                    self._save()

    def _poll_goals(self, goals, wall):
        changed = False
        for rid, goal in list(goals.items()):
            client = self.node.cancel_clients[(rid, 'rescue')]
            if client.service_is_ready():
                request = CancelGoal.Request()
                request.goal_info.goal_id = UUID(uuid=goal.uuid)
                client.call_async(request)
            if self._terminal(rid, goal, wall):
                del goals[rid]
                changed = True
        return changed

    def _terminal(self, rid, goal, wall):
        client, service_type = self.node.result_clients[(rid, 'rescue')]
        if goal.probe is None:
            if client.service_is_ready():
                goal.probe = client.call_async(service_type.Request(goal_id=UUID(uuid=goal.uuid)))
                goal.probe_since = wall
            return False
        if not goal.probe.done():
            if wall - goal.probe_since >= 2:
                goal.probe.cancel()
                goal.probe = None
            return False
        result, goal.probe = goal.probe, None
        status = result.result().status
        if status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                      GoalStatus.STATUS_CANCELED):
            return True
        if status == GoalStatus.STATUS_UNKNOWN:
            goal.unknown_at = goal.unknown_at or wall
            robot = self.node.robots.get(rid)
            return bool(wall - goal.unknown_at >= 2 and robot and robot.idle(
                self.node._now(), self.node.config.settings.state_stale_s))
        return False

    def _verify(self, assignment, now):
        verifier = assignment.verifier
        task = assignment.task
        ids = task.assigned_robot_ids
        group = [self.node.robots[rid] for rid in ids if rid in self.node.robots]
        valid = len(group) == len(ids) and all(
            r.mode == RobotState.MODE_HOLDING and r.active_task == task.task_id and r.stationary()
            for r in group)
        previous = verifier.start
        done, reset, _ = verifier.update(
            now, group if valid else [], assignment.target, self.node.config.settings, valid,
            zones=[s[:2] for s in assignment.target.slots])
        if reset:
            self._event(assignment, RescueEvent.EVENT_DWELL_RESET,
                        TaskReason(code=TaskReason.DWELL_RESET))
        if verifier.start is not None and (previous is None or reset):
            self._event(assignment, RescueEvent.EVENT_DWELL_STARTED)
        if done:
            self._finish(assignment, 'complete')

    def _finish(self, assignment, kind, code=TaskReason.NONE, detail='', robot=None):
        if assignment.ending:
            return
        interrupt = self._interrupt()
        if interrupt:
            self.abort(interrupt)
            return
        reason = TaskReason(code=code, detail=detail)
        types = {'complete': RescueEvent.EVENT_COMPLETED, 'fail': RescueEvent.EVENT_FAILED,
                 'cancel': RescueEvent.EVENT_CANCELLED}
        self._event(assignment, types[kind], reason)
        assignment.ending = True
        task = assignment.task
        if kind == 'complete':
            self.retry_avoid.pop(task.target_id, None)
        if task.scoring_stage == RescueTask.STAGE_TASK2B:
            self._apply(kind, assignment, reason)
        else:
            tid = task.target_id
            if kind == 'fail':
                self.retry_at[tid] = time.monotonic() + self.retry_backoff
                self.retry_avoid[tid] = {robot} if robot else set(task.assigned_robot_ids)
                retries = self.navigation_retries.get(tid, 0)
                if (code == TaskReason.NAVIGATION_FAILED and robot is not None
                        and retries < NAVIGATION_RETRIES):
                    self.navigation_retries[tid] = retries + 1
                    self.basic_attempts[tid] -= 1
                elif self.basic_attempts[tid] >= self.node.config.settings.max_attempts:
                    self.basic_failed.add(tid)
            self._save()

    def _event(self, assignment, event_type, reason=None):
        task = assignment.task
        reason = reason or TaskReason()
        now = self.node.get_clock().now().to_msg()
        index = list(range(len(task.assigned_robot_ids)))
        raw = self.node.raw_states
        poses = [raw[r].pose.pose if r in raw else Pose()
                 for r in task.assigned_robot_ids]
        saved = self.journal.event({
            'type': {RescueEvent.EVENT_ASSIGNED: 'ASSIGNED',
                     RescueEvent.EVENT_DWELL_STARTED: 'DWELL_STARTED',
                     RescueEvent.EVENT_DWELL_RESET: 'DWELL_RESET',
                     RescueEvent.EVENT_COMPLETED: 'COMPLETED', RescueEvent.EVENT_FAILED: 'FAILED',
                     RescueEvent.EVENT_CANCELLED: 'CANCELLED'}[event_type],
            'sim_time': seconds(now), 'scoring_stage': task.scoring_stage,
            'mission_id': task.mission_id, 'step_id': task.step_id,
            'target_id': task.target_id, 'task_id': task.task_id,
            'robot_ids': list(task.assigned_robot_ids), 'robot_poses': [to_dict(p) for p in poses],
            'zone_indices': index, 'zone_centers': [s[:2] for s in assignment.target.slots],
            'zone_tolerance_m': task.zone_tolerance_m, 'required_dwell_s': assignment.target.dwell,
            'dwell_elapsed_s': assignment.verifier.elapsed, 'reason': to_dict(reason),
        })
        if not saved:
            return
        msg = RescueEvent(
            run_id=task.run_id, event_id=saved['event_id'], task_id=task.task_id,
            target_id=task.target_id, mission_id=task.mission_id, step_id=task.step_id,
            scoring_stage=task.scoring_stage, event_type=event_type,
            robot_ids=task.assigned_robot_ids, robot_poses=poses, rescue_zone_indices=index,
            reason=reason)
        msg.header.frame_id = 'map'
        msg.header.stamp = now
        ns = round(assignment.verifier.elapsed * 1e9)
        msg.dwell_elapsed.sec, msg.dwell_elapsed.nanosec = divmod(ns, 10**9)
        self.node.event_pub.publish(msg)

    def _record_contacts(self):
        contacts = self.node.collision.events
        while contacts:
            self.journal.event({'type': 'COLLISION', **contacts.pop(0)})
            self._save()

    def on_rescue_event(self, event):
        """Accept injected outcomes only in the explicit ROS test harness."""
        a = self.assigned.get(event.task_id)
        if a is None or event.scoring_stage != RescueEvent.STAGE_TASK2B:
            return
        task = a.task
        if ((event.run_id, event.mission_id, event.step_id, event.target_id)
                != (task.run_id, task.mission_id, task.step_id, task.target_id)
                or event.robot_ids != task.assigned_robot_ids):
            return
        kind = {RescueEvent.EVENT_COMPLETED: 'complete', RescueEvent.EVENT_FAILED: 'fail',
                RescueEvent.EVENT_CANCELLED: 'cancel'}.get(event.event_type)
        if time.monotonic() >= a.deadline:
            self._finish(a, 'fail', TaskReason.NAVIGATION_TIMEOUT)
        elif kind:
            self._finish(a, kind, event.reason.code, event.reason.detail)
