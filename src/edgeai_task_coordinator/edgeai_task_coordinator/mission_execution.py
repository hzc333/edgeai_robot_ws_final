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

"""Connect the Task 2B dependency scheduler to robot actions and verifier events."""

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
import uuid

from action_msgs.srv import CancelGoal
from edgeai_common.config import resolve_parameters
from edgeai_common.qos import qos_from_contract
from edgeai_rescue_coordinator.ros_support import message, to_dict
from edgeai_rescue_coordinator.scheduler import DependencyScheduler
from edgeai_robot_interfaces.action import ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState, MissionExecutionState, MissionPlan, RescueEvent, RescueTask, TaskReason,
)
from geometry_msgs.msg import PoseStamped
from unique_identifier_msgs.msg import UUID


@dataclass
class Assignment:
    """Keep reservations until the robot action results confirm release."""

    task: RescueTask
    deadline: float
    goals: dict = field(default_factory=dict)
    ending: bool = False
    retry_cancel_at: float = 0.0


class MissionExecution:
    """Run one validated plan at a time in the coordinator's serial executor."""

    def __init__(self, coordinator, contract, target_path, step_timeout):
        if not math.isfinite(step_timeout) or step_timeout <= 0:
            raise ValueError('task2b_step_timeout_s must be finite and positive')
        self.node = coordinator
        self.step_timeout = step_timeout
        config_dir = coordinator.get_parameter('config_dir').value or None
        self.rescue_parameters = resolve_parameters('development', config_dir).values['rescue']
        self.registry = json.loads(Path(target_path).read_text())
        if self.registry['frame_id'] != 'map' or self.registry['map']['frame_id'] != 'map':
            raise ValueError('Task 2B targets must be in the map frame')
        self.context = (self.registry['run_id'], self.registry['map']['map_id'],
                        self.registry['map']['revision'], self.registry['revision'])
        self.targets = {t['target_id']: t for t in self.registry['targets']}
        self.scheduler = None
        self.pending = None
        self.seen = set()
        self.assigned = {}
        topics = contract['topics']
        snapshot = coordinator.qos['snapshot']
        self.state_pub = coordinator.create_publisher(
            MissionExecutionState, topics['mission_state'], snapshot)
        assignment_qos = qos_from_contract(contract, 'snapshot')
        assignment_qos.depth = len(coordinator.config.robots)
        self.tasks_pub = coordinator.create_publisher(
            RescueTask, topics['rescue_tasks'], assignment_qos)
        coordinator.create_subscription(MissionPlan, topics['mission_plan'], self.on_plan, snapshot)
        coordinator.create_subscription(
            RescueEvent, topics['rescue_events'], self.on_rescue_event,
            coordinator.qos['event_stream'])

    def on_plan(self, plan):
        """Accept plans only for the loaded frozen target snapshot."""
        if (not plan.valid or plan.mission_id in self.seen
                or self._context(plan) != self.context):
            return
        if self.pending is not None or self._active() or self.assigned:
            self.node.get_logger().warning(
                f'Already executing a mission; skipped {plan.mission_id}')
            return
        self.pending = plan

    def _active(self):
        return self.scheduler is not None and self.scheduler.snapshot()['state'] in (
            MissionExecutionState.STATE_IDLE, MissionExecutionState.STATE_EXECUTING)

    @staticmethod
    def _context(msg):
        return msg.run_id, msg.map_id, msg.map_revision, msg.target_revision

    def _publish(self):
        self.state_pub.publish(message(MissionExecutionState, self.scheduler.snapshot()))

    def _apply(self, kind, assignment=None, reason=None):
        snapshot = self.scheduler.snapshot()
        # A clock rewind must not prevent aborting and releasing robot actions.
        stamp = to_dict(self.node.get_clock().now().to_msg())
        previous = snapshot['header']['stamp']
        if (stamp['sec'], stamp['nanosec']) < (previous['sec'], previous['nanosec']):
            stamp = previous
        event = {'kind': kind, 'stamp': stamp, 'run_id': snapshot['run_id'],
                 'mission_id': snapshot['mission_id']}
        if assignment is not None:
            task = assignment.task
            event.update(step_id=task.step_id, task_id=task.task_id,
                         robot_ids=list(task.assigned_robot_ids))
        if reason is not None:
            event['reason'] = to_dict(reason)
        if not self.scheduler.apply(event):
            return False
        self._publish()
        return True

    def _interrupt(self):
        node = self.node
        phase = node.phase
        if (node.shutdown_requested or node.estop or node.last_error
                or (phase and (phase.estop_active or phase.phase == phase.PHASE_ESTOP))):
            return 'safety_stop'
        if (phase is None or phase.phase != CompetitionState.PHASE_TASK2B or phase.frozen
                or phase.reset_in_progress
                or phase.run_mode != CompetitionState.RUN_MODE_DEVELOPMENT
                or self._context(phase) != self.context):
            return 'phase_changed'
        if time.monotonic() - node.phase_received > node.config.settings.phase_heartbeat_timeout_s:
            return 'safety_stop'
        return None

    def abort(self, kind='safety_stop'):
        """Stop dispatch and release every reservation through the action protocol."""
        self.pending = None
        if self._active():
            self._apply(kind)
        for assignment in self.assigned.values():
            assignment.ending = True

    def tick(self, now):
        """Advance safety, action release, timeout and dispatch on the steady timer."""
        interrupt = self._interrupt()
        if interrupt and self.scheduler is not None:
            self.abort(interrupt)
        for task_id, assignment in list(self.assigned.items()):
            if not assignment.ending and time.monotonic() >= assignment.deadline:
                self._finish(assignment, 'fail', TaskReason.NAVIGATION_TIMEOUT,
                             'Task 2B step exceeded its execution timeout')
            if assignment.ending:
                self._cancel(assignment)
                if not assignment.goals:
                    del self.assigned[task_id]
        if interrupt or self.node.goal_records or now <= 0:
            return
        if self.pending is not None and not self.assigned:
            plan, self.pending = self.pending, None
            self.seen.add(plan.mission_id)
            try:
                for step in plan.steps:
                    self._target_zones(step.target_id)
                payload = to_dict(plan)
                payload['header']['stamp'] = to_dict(self.node.get_clock().now().to_msg())
                self.scheduler = DependencyScheduler(payload)
            except (KeyError, TypeError, ValueError) as error:
                self.node.get_logger().error(f'Unusable Task 2B plan {plan.mission_id}: {error}')
                return
            self._publish()
        if self._active():
            self._dispatch_ready(now)

    def _target_zones(self, target_id):
        target = self.targets[target_id]
        zones = sorted(target['rescue_zones'], key=lambda zone: zone['index'])
        count = {'normal': 1, 'cooperative': 2}[target['target_type']]
        if [zone['index'] for zone in zones] != list(range(count)):
            raise ValueError(f'incorrect rescue zones for {target_id}')
        return count, zones

    def _dispatch_ready(self, now):
        node = self.node
        reserved = {rid for a in self.assigned.values() for rid in a.task.assigned_robot_ids}
        occupied_targets = {a.task.target_id for a in self.assigned.values()}
        for step in self.scheduler.ready_steps():
            target_id = step['target_id']
            if target_id in occupied_targets:
                continue
            count, zones = self._target_zones(target_id)
            available = {
                rid for rid, robot in node.robots.items()
                if rid not in reserved and node.raw_states[rid].run_id == self.context[0]
                and robot.idle(now, node.config.settings.state_stale_s)
                and node.action_clients[(rid, 'rescue')].server_is_ready()
            }
            if len(available) < count:
                continue
            robots = []
            for zone in zones:
                point = zone['pose']['position']
                rid = min(available, key=lambda r: (
                    math.hypot(node.robots[r].x - point['x'], node.robots[r].y - point['y']), r))
                robots.append(rid)
                available.remove(rid)
            reserved.update(robots)
            occupied_targets.add(target_id)
            self._start(step, robots, zones)

    def _start(self, step, robots, zones):
        node = self.node
        snapshot = self.scheduler.snapshot()
        stamp = node.get_clock().now().to_msg()
        poses = [message(PoseStamped, {'header': {'frame_id': 'map', 'stamp': to_dict(stamp)},
                                       'pose': zone['pose']}) for zone in zones]
        task = RescueTask(
            run_id=self.context[0], map_id=self.context[1], map_revision=self.context[2],
            task_id=uuid.uuid4().hex, target_id=step['target_id'],
            mission_id=snapshot['mission_id'], step_id=step['step_id'],
            scoring_stage=RescueTask.STAGE_TASK2B,
            task_type=RescueTask.TYPE_NORMAL if len(robots) == 1 else RescueTask.TYPE_COOPERATIVE,
            state=RescueTask.STATE_ASSIGNED, priority=step['priority'], assigned_robot_ids=robots,
            goal_poses=poses, zone_tolerance_m=self.rescue_parameters['position_tolerance_m'])
        task.header.frame_id = 'map'
        task.header.stamp = stamp
        dwell_key = 'normal_dwell_s' if len(robots) == 1 else 'cooperative_dwell_s'
        dwell_ns = round(self.rescue_parameters[dwell_key] * 1e9)
        task.required_dwell.sec, task.required_dwell.nanosec = divmod(dwell_ns, 10**9)
        predecessors = set(step['predecessor_step_ids'])
        task.predecessor_task_ids = [s['task_id'] for s in snapshot['steps']
                                     if s['step_id'] in predecessors]
        assignment = Assignment(task, time.monotonic() + self.step_timeout)
        if not self._apply('start', assignment):
            raise RuntimeError(f'Dependency scheduler rejected start for {task.step_id}')
        self.assigned[task.task_id] = assignment
        self.tasks_pub.publish(task)
        for index, rid in enumerate(robots):
            goal = ExecuteRescueTask.Goal(
                run_id=task.run_id, map_id=task.map_id, map_revision=task.map_revision,
                task_id=task.task_id, target_id=task.target_id, target_type=task.task_type,
                rescue_zone_index=index, rescue_pose=poses[index],
                arrival_xy_tolerance_m=self.rescue_parameters['arrival_xy_tolerance_m'])
            goal_id = UUID(uuid=list(uuid.uuid4().bytes))
            assignment.goals[rid] = goal_id
            try:
                client = node.action_clients[(rid, 'rescue')]
                future = client.send_goal_async(goal, goal_uuid=goal_id)
                future.add_done_callback(
                    lambda f, aid=assignment, robot=rid: self._accepted(aid, robot, f))
            except Exception as error:
                self._finish(assignment, 'fail', TaskReason.INTERNAL_ERROR, str(error))
                break

    def _accepted(self, assignment, robot, future):
        try:
            handle = future.result()
            if handle.accepted:
                handle.get_result_async().add_done_callback(
                    lambda f: self._ended(assignment, robot, f))
            else:
                assignment.goals.pop(robot, None)
                self._finish(assignment, 'fail', TaskReason.NAVIGATION_FAILED,
                             f'{robot} rejected the rescue action')
        except Exception as error:
            self._finish(assignment, 'fail', TaskReason.INTERNAL_ERROR, str(error))

    def _ended(self, assignment, robot, future):
        try:
            result = future.result().result
        except Exception as error:
            self._finish(assignment, 'fail', TaskReason.INTERNAL_ERROR, str(error))
            return
        assignment.goals.pop(robot, None)
        code = result.reason.code or TaskReason.NAVIGATION_FAILED
        self._finish(assignment, 'fail', code,
                     f'{robot} action ended before verifier completion: {result.reason.detail}')

    def _cancel(self, assignment):
        now = time.monotonic()
        if now < assignment.retry_cancel_at:
            return
        assignment.retry_cancel_at = now + 0.5
        for rid, goal_id in assignment.goals.items():
            client = self.node.cancel_clients[(rid, 'rescue')]
            if client.service_is_ready():
                request = CancelGoal.Request()
                request.goal_info.goal_id = goal_id
                client.call_async(request)

    def _finish(self, assignment, kind, code=TaskReason.NONE, detail=''):
        if not assignment.ending:
            interrupt = self._interrupt()
            if interrupt:
                self.abort(interrupt)
            else:
                reason = TaskReason(code=code, detail=detail)
                if not self._apply(kind, assignment, reason):
                    self.abort()
                assignment.ending = True

    def on_rescue_event(self, event):
        """Only a matching Task 2B verifier outcome can complete a step."""
        assignment = self.assigned.get(event.task_id)
        if assignment is None or event.scoring_stage != RescueEvent.STAGE_TASK2B:
            return
        task = assignment.task
        if ((event.run_id, event.mission_id, event.step_id, event.target_id)
                != (task.run_id, task.mission_id, task.step_id, task.target_id)
                or event.robot_ids != task.assigned_robot_ids):
            return
        if time.monotonic() >= assignment.deadline:
            self._finish(assignment, 'fail', TaskReason.NAVIGATION_TIMEOUT,
                         'Verifier outcome arrived after step timeout')
            return
        kind = {RescueEvent.EVENT_COMPLETED: 'complete', RescueEvent.EVENT_FAILED: 'fail',
                RescueEvent.EVENT_CANCELLED: 'cancel'}.get(event.event_type)
        if kind:
            self._finish(assignment, kind, event.reason.code, event.reason.detail)
