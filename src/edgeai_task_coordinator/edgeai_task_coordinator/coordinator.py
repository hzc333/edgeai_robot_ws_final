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

"""ROS adapter for the official midterm; never owns phase, TF or velocity."""

from concurrent.futures import ThreadPoolExecutor
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time
import uuid

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.action import ExecuteNavigationGoal, ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState, EmergencyStop, RescueEvent, RobotState, RobotStateArray, TaskReason,
)
from edgeai_robot_interfaces.srv import SetCompetitionPhase
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from unique_identifier_msgs.msg import UUID

from .collision import CollisionState
from .engine import Command, Engine
from .evidence import Journal
from .model import load_configuration, Robot
from .physics import ContactFeed
from .planning import GridCost
from .scheduling import footprint, polygon_gap


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def message_robot(msg):
    """Untrusted robot messages become finite map/base_footprint samples."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    valid_q = (all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w))
               and abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1) <= 0.01)
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Robot(msg.robot_id, seconds(msg.header.stamp), p.x, p.y,
                 localized=msg.localized and msg.header.frame_id == 'map' and valid_q,
                 navigation_available=msg.navigation_available, mode=msg.mode,
                 active_task=msg.active_task_id,
                 fault=any(f.severity >= 2 for f in msg.faults),
                 linear=msg.velocity.twist.linear.x, angular=msg.velocity.twist.angular.z,
                 yaw=yaw)


class Coordinator(Node):
    """Single-threaded scheduling avoids races between phase and action callbacks."""

    def __init__(self, *, parameter_overrides=None):
        super().__init__('task_allocator', namespace='/edge',
                         parameter_overrides=parameter_overrides)
        default = str(Path(get_package_share_directory('edgeai_task_coordinator'))
                      / 'config' / 'midterm.yaml')
        self.declare_parameter('config_file', default)
        self.declare_parameter('config_dir', '')
        self.declare_parameter('output_dir', str(Path.cwd() / 'runs' / 'p4'))
        self.declare_parameter('require_map', True)
        self.declare_parameter('test_mode', False)
        self.declare_parameter('request_finish', True)
        self.declare_parameter('require_collision_monitor', True)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        if (not self.get_parameter('use_sim_time').value
                and not self.get_parameter('test_mode').value):
            raise ValueError('official midterm holds require use_sim_time=true')
        if (not self.get_parameter('require_map').value
                and not self.get_parameter('test_mode').value):
            raise ValueError('only test_mode may bypass the shared map requirement')
        if (not self.get_parameter('require_collision_monitor').value
                and not self.get_parameter('test_mode').value):
            raise ValueError('only test_mode may bypass physical collision monitoring')
        self.config = load_configuration(self.get_parameter('config_file').value)
        require_replay = self.get_parameter('require_collision_monitor').value
        self.collision = CollisionState(self.config.robots, require_replay=require_replay)
        self.contact_feed = (ContactFeed()
                             if self.get_parameter('require_collision_monitor').value else None)
        self.config_hash = hashlib.sha256(json.dumps(asdict(self.config), sort_keys=True)
                                          .encode()).hexdigest()
        contract = load_runtime_contract(self.get_parameter('config_dir').value or None)
        self.qos = {name: qos_from_contract(contract, name)
                    for name in ('robot_state', 'snapshot', 'control_state', 'event_stream')}
        self.registry = Node('robot_state_registry', namespace='/edge')
        self.verifier_node = Node('rescue_verifier', namespace='/edge')
        self.aggregate_pub = self.registry.create_publisher(
            RobotStateArray, '/edge/robot_states', self.qos['robot_state'])
        self.event_pub = self.verifier_node.create_publisher(
            RescueEvent, '/edge/rescue_events', self.qos['event_stream'])
        self.status_pub = self.create_publisher(
            String, '/edge/midterm_rescue/status', self.qos['snapshot'])
        self.collision_pub = self.create_publisher(
            String, '/edge/midterm_rescue/collision', self.qos['snapshot'])
        self.create_subscription(CompetitionState, '/competition/state', self._on_phase,
                                 self.qos['control_state'])
        self.create_subscription(EmergencyStop, '/competition/estop', self._on_estop,
                                 self.qos['control_state'])
        self.create_subscription(OccupancyGrid, '/map', self._on_map, self.qos['snapshot'])
        self.action_clients = {}
        self.cancel_clients = {}
        self.result_clients = {}
        self.phase_client = self.create_client(SetCompetitionPhase, '/competition/set_phase')
        self.finish_request = 'NOT_REQUESTED'
        self.finish_future = None
        self.safety_future = None
        for robot_id in self.config.robots:
            self.create_subscription(
                RobotState, f'/{robot_id}/state',
                lambda msg, rid=robot_id: self._on_robot(msg, rid), self.qos['robot_state'])
            for kind, action_type, suffix in (
                    ('rescue', ExecuteRescueTask, 'execute_rescue_task'),
                    ('navigation', ExecuteNavigationGoal, 'execute_navigation_goal')):
                action = f'/{robot_id}/{suffix}'
                self.action_clients[(robot_id, kind)] = ActionClient(self, action_type, action)
                self.cancel_clients[(robot_id, kind)] = self.create_client(
                    CancelGoal, action + '/_action/cancel_goal')
                self.result_clients[(robot_id, kind)] = (
                    self.create_client(action_type.Impl.GetResultService,
                                       action + '/_action/get_result'),
                    action_type.Impl.GetResultService)
        self.robots, self.raw_states = {}, {}
        self.phase = None
        self.phase_received = None
        self.estop_generation = -1
        self.estop = False
        self.grid = None
        self.map_worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='p4-map')
        self.map_future = None
        self.map_fingerprint = None
        self.pending_map = None
        self.engine = None
        self.journal = None
        self.identity = None
        self.handles = {}
        self.goal_records = {}
        self.cancel_pending = set()
        self.last_error = ''
        self.transport_warning = ''
        self.last_sample = None
        self.recovered = False
        self.shutdown_requested = False
        self.orphaned = set()
        self.orphan_futures = {}
        self.resume_after_sequence = -1
        self.recovery_ready_since = None
        self.recovery_gate = ''
        self.last_ros_time = None
        self.instance_id = uuid.uuid4().hex
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self._tick, clock=self.steady_clock)

    def _on_robot(self, msg, robot_id):
        if msg.robot_id != robot_id or (self.phase and msg.run_id != self.phase.run_id):
            return
        previous = self.robots.get(robot_id)
        robot = message_robot(msg)
        if previous is not None and robot.stamp < previous.stamp:
            return
        self.robots[robot_id], self.raw_states[robot_id] = robot, msg

    def _on_phase(self, msg):
        if self.phase and msg.run_id == self.phase.run_id and \
                msg.phase_sequence < self.phase.phase_sequence:
            return
        if self.phase and msg.run_id == self.phase.run_id and \
                msg.phase_sequence == self.phase.phase_sequence and (
                    msg.phase, msg.map_id, msg.map_revision, msg.frozen,
                    msg.estop_active, msg.reset_in_progress) != (
                        self.phase.phase, self.phase.map_id, self.phase.map_revision,
                        self.phase.frozen, self.phase.estop_active, self.phase.reset_in_progress):
            self.last_error = 'CONFLICTING_PHASE_SEQUENCE'
            return
        if self.phase and msg.run_id != self.phase.run_id:
            self.robots.clear()
            self.raw_states.clear()
        self.phase = msg
        self.phase_received = time.monotonic()

    def _on_estop(self, msg):
        if msg.generation > self.estop_generation:
            self.estop_generation, self.estop = msg.generation, msg.active

    def _rescue_phase(self):
        # P2 keeps mapping during TASK1 and bumps map_revision when it freezes the map:
        # take the map and the run identity only from TASK2A on (P1 integration).
        return bool(self.phase and self.phase.phase in (
            CompetitionState.PHASE_TASK2A, CompetitionState.PHASE_FROZEN,
            CompetitionState.PHASE_TASK2B))

    def _on_map(self, msg):
        if not self._rescue_phase():
            self.pending_map = msg
            return
        q = msg.info.origin.orientation
        values = (msg.info.resolution, msg.info.origin.position.x,
                  msg.info.origin.position.y, q.x, q.y, q.z, q.w)
        if (not all(math.isfinite(v) for v in values)
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.01):
            self.last_error = 'INVALID_MAP_GEOMETRY'
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if (msg.header.frame_id != 'map' or msg.info.resolution <= 0
                or msg.info.width * msg.info.height != len(msg.data)):
            self.last_error = 'INVALID_MAP'
            return
        fingerprint = (tuple(values), msg.info.width, msg.info.height, hash(tuple(msg.data)))
        if fingerprint == self.map_fingerprint:
            return
        self.map_fingerprint = fingerprint
        self.grid = None
        width, height, resolution = msg.info.width, msg.info.height, msg.info.resolution
        origin = (msg.info.origin.position.x, msg.info.origin.position.y, yaw)
        data = tuple(msg.data)
        points = [p[:2] for t in self.config.targets for p in (*t.slots, *t.approaches)]

        def prepare():
            return GridCost(width, height, resolution, origin, data,
                            self.config.settings.allocation_clearance_m).validate_parking(
                                self.config.targets, self.config.settings).prepare(points)

        self.map_future = self.map_worker.submit(prepare)

    def _enabled(self, now):
        p = self.phase
        return bool(p and p.phase == p.PHASE_TASK2A and not p.frozen and not p.estop_active
                    and not p.reset_in_progress and not self.estop and not self.last_error
                    and not self.shutdown_requested
                    and now > 0 and -0.1 <= now - seconds(p.header.stamp) <= 2.0
                    and time.monotonic() - self.phase_received <=
                    self.config.settings.phase_heartbeat_timeout_s
                    and (self.grid is not None or not self.get_parameter('require_map').value)
                    and (self.contact_feed is None or (
                        self.collision.snapshot(now, time.monotonic())['state'] == 'CLEAR'
                        and not self.collision.count)))

    def _start(self):
        p = self.phase
        self.identity = {'run_id': p.run_id, 'map_id': p.map_id,
                         'map_revision': p.map_revision, 'config_sha256': self.config_hash,
                         'rule_profile': self.config.profile,
                         'target_source': 'official_midterm_coordinates',
                         'rule_source': 'KTH_EdgeAI_Midterm_Check_Task_Description.docx'}
        self.journal = Journal(self.get_parameter('output_dir').value, self.identity)
        if self.journal.monitor_started is None:
            self.journal.event({'type': 'MONITOR_STARTED', 'sim_time': self._now()})
        self.collision.monitor_started = self.journal.monitor_started
        self.engine = Engine(self.config, p.run_id, self.journal.completed)
        while self.engine.target and self.engine.target.target_id in self.engine.completed:
            self.engine.index += 1
        previous = self.journal.previous
        if previous:
            self.collision.restore(previous.get('collision_history', {}))
            self.engine.attempts = dict(previous.get('attempts', {}))
            self.engine.failures = dict(previous.get('failures', {}))
            self.engine.retry_avoid = dict(previous.get('retry_avoid', {}))
            self.engine.retry_not_before = previous.get('retry_not_before', 0.0)
            self.resume_after_sequence = previous.get('resume_after_sequence', -1)
            self.goal_records = previous.get('goals', {})
            if previous['status'] in ('ACTIVE', 'RELEASING', 'RECOVERING') or self.goal_records:
                self.recovered = True
                self.engine.status = 'RECOVERING'
                self.engine.recovery_required = True
                self.engine.failure = 'RESTART_CLEANUP'
                self.orphaned = set(self.goal_records)
                for token in self.goal_records:
                    self.goal_records[token].pop('last_cancel', None)
                    self.goal_records[token].pop('unknown_since', None)
                    self.cancel_pending.add(token)
            elif previous['status'] == 'HALTED':
                self.engine.status = 'HALTED'
                self.engine.failure = previous.get('failure', 'PREVIOUS_FATAL_FAILURE')
        if self.journal.contacts:
            last = self.journal.contacts[-1]
            instance = last.get('physics_instance', self.collision.observer_instance)
            sequence = max((c.get('sequence', 0) for c in self.journal.contacts
                            if c.get('physics_instance') == instance), default=0)
            if instance == self.collision.observer_instance:
                sequence = max(sequence, self.collision.last_sequence or 0)
            self.collision.restore({**self.collision.history(),
                                    'observer_instance': instance, 'last_sequence': sequence,
                                    'count': len(self.journal.contacts),
                                    'first': self.journal.contacts[0],
                                    'last': self.journal.contacts[-1]})
        self.get_logger().info(f'P4 evidence: {self.journal.directory}')

    def _send(self, command):
        robot = self.robots.get(command.robot_id)
        slot = self.engine.target.slots[command.slot]
        already_parked = (robot and math.dist((robot.x, robot.y), slot[:2])
                          <= self.config.settings.arrival_tolerance_m)
        if self.engine.target.approaches and not already_parked:
            self._send_navigation(command)
        else:
            self._send_rescue(command)

    def _send_navigation(self, command):
        engine = self.engine
        x, y, yaw = engine.target.approaches[command.slot]
        goal = ExecuteNavigationGoal.Goal(
            run_id=engine.run_id, request_id=engine.task_id,
            map_id=self.identity['map_id'], map_revision=self.identity['map_revision'],
            purpose=ExecuteNavigationGoal.Goal.PURPOSE_TRANSIT,
            xy_tolerance_m=0.10, yaw_tolerance_rad=0.25)
        goal.timeout.sec = int(self.config.settings.executor_timeout_wall_s)
        goal.goal_pose.header.frame_id = 'map'
        goal.goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = x, y
        goal.goal_pose.pose.orientation.z = math.sin(yaw / 2)
        goal.goal_pose.pose.orientation.w = math.cos(yaw / 2)
        self._dispatch(command, goal, 'navigation')

    def _send_rescue(self, command):
        engine = self.engine
        target = engine.target
        x, y, yaw = target.slots[command.slot]
        goal = ExecuteRescueTask.Goal(
            run_id=engine.run_id, task_id=engine.task_id, target_id=target.target_id,
            map_id=self.identity['map_id'], map_revision=self.identity['map_revision'],
            target_type=1 if target.required == 1 else 2, rescue_zone_index=command.slot,
            arrival_xy_tolerance_m=self.config.settings.arrival_tolerance_m)
        goal.rescue_pose.header.frame_id = 'map'
        goal.rescue_pose.header.stamp = self.get_clock().now().to_msg()
        goal.rescue_pose.pose.position.x, goal.rescue_pose.pose.position.y = x, y
        goal.rescue_pose.pose.orientation.z = math.sin(yaw / 2)
        goal.rescue_pose.pose.orientation.w = math.cos(yaw / 2)
        self._dispatch(command, goal, 'rescue')

    def _dispatch(self, command, goal, kind):
        client = self.action_clients[(command.robot_id, kind)]
        if not client.server_is_ready():
            self.engine.accepted(command.token, False, self._now())
            return
        goal_uuid = UUID(uuid=list(uuid.uuid4().bytes))
        self.goal_records[command.token] = {
            'robot_id': command.robot_id, 'uuid': [int(v) for v in goal_uuid.uuid], 'kind': kind}
        self._save()  # Write-ahead receipt, even if the acceptance reply is lost.
        future = client.send_goal_async(goal, goal_uuid=goal_uuid)
        future.add_done_callback(lambda f, token=command.token: self._accepted(token, f, kind))

    def _accepted(self, token, future, kind):
        try:
            handle = future.result()
        except Exception as error:
            self.transport_warning = f'ACTION_TRANSPORT: {error}'
            self.cancel_pending.add(token)
            self.orphaned.add(token)
            self.engine.interrupt('ACTION_TRANSPORT_UNCERTAIN', self._now())
            return
        if handle.accepted:
            self.handles[token] = handle
            handle.get_result_async().add_done_callback(
                lambda f, tok=token: self._ended(tok, f, kind))
        else:
            self.goal_records.pop(token, None)
        callback = self.engine.transit_accepted if kind == 'navigation' else self.engine.accepted
        callback(token, handle.accepted, self._now())
        if handle.accepted and (token not in self.engine.slots or
                                self.engine.status in ('RELEASING', 'HALTED', 'RECOVERING')):
            self.cancel_pending.add(token)

    def _ended(self, token, future, kind):
        success = False
        try:
            response = future.result()
            reason = f'ACTION_ENDED:{response.result.reason.code}:{response.result.reason.detail}'
            success = kind == 'navigation' and response.result.success
        except Exception as error:
            reason = f'ACTION_RESULT_ERROR:{error}'
        self.handles.pop(token, None)
        self.goal_records.pop(token, None)
        self.cancel_pending.discard(token)
        if success and self.engine.status == 'ACTIVE':
            self.engine.transit_finished(token)
        else:
            self.engine.ended(token, self._now(), reason)

    def _cancel(self, token):
        record = self.goal_records.get(token)
        if not record:
            self.cancel_pending.discard(token)
            return
        client = self.cancel_clients[(record['robot_id'], record.get('kind', 'rescue'))]
        if not client.service_is_ready():
            return
        if time.monotonic() - record.get('last_cancel', -100.0) < 0.5:
            return
        request = CancelGoal.Request()
        request.goal_info.goal_id = UUID(uuid=record['uuid'])
        record['last_cancel'] = time.monotonic()
        # Keep retrying until the action result and idle state confirm release.
        client.call_async(request)
        if token in self.orphaned:
            self._reconcile_orphan(token, record)

    def _reconcile_orphan(self, token, record):
        client, service_type = self.result_clients[
            (record['robot_id'], record.get('kind', 'rescue'))]
        future = self.orphan_futures.get(token)
        if future is None and client.service_is_ready():
            request = service_type.Request(goal_id=UUID(uuid=record['uuid']))
            record['result_requested_wall'] = time.monotonic()
            self.orphan_futures[token] = client.call_async(request)
        elif future is not None and not future.done():
            if time.monotonic() - record.get('result_requested_wall', 0) > 2.0:
                future.cancel()
                self.orphan_futures.pop(token, None)
        elif future is not None and future.done():
            response = future.result()
            self.orphan_futures.pop(token, None)
            terminal = response.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                           GoalStatus.STATUS_CANCELED)
            # A write-ahead UUID may never have been sent. Require two UNKNOWN
            # replies at least a wall second apart, then fresh idle state too.
            if response.status == GoalStatus.STATUS_UNKNOWN:
                first = record.setdefault('unknown_since', time.monotonic())
                terminal = time.monotonic() - first >= 1.0
            if terminal:
                self.goal_records.pop(token, None)
                self.cancel_pending.discard(token)
                self.orphaned.discard(token)
                self.engine.ended(token, self._now(), 'RECONCILED_ACTION_RESULT')

    def _try_recover(self, now, enabled):
        if self.engine.status != 'RECOVERING':
            self.recovery_ready_since = None
            self.recovery_gate = ''
            return
        if self.contact_feed is not None and self.collision.count:
            self.recovery_gate = 'PHYSICAL_CONTACT_RECORDED'
        elif self.contact_feed is not None and self.collision.snapshot(
                now, time.monotonic())['state'] != 'CLEAR':
            self.recovery_gate = 'WAITING_FOR_PHYSICS_OBSERVER'
        elif not enabled or self.phase.phase_sequence <= self.resume_after_sequence:
            self.recovery_gate = 'WAITING_FOR_TASK2A_AUTHORIZATION'
        elif self.goal_records or self.handles:
            self.recovery_gate = 'WAITING_FOR_OLD_ACTION_RESULTS'
        elif not all(r in self.robots and self.robots[r].idle(
                now, self.config.settings.state_stale_s) for r in self.config.robots):
            self.recovery_gate = 'WAITING_FOR_FRESH_IDLE_ROBOTS'
        elif polygon_gap(*(footprint(self.robots[r].x, self.robots[r].y,
                                     self.robots[r].yaw, self.config.settings)
                           for r in self.config.robots)) <= self.config.settings.entry_clearance_m:
            self.recovery_gate = 'WAITING_FOR_FLEET_CLEARANCE'
        elif not all(c.server_is_ready() for c in self.action_clients.values()):
            self.recovery_gate = 'WAITING_FOR_ACTION_SERVERS'
        else:
            self.recovery_gate = 'CONFIRMING_STABLE_INPUTS'
            if self.recovery_ready_since is None:
                self.recovery_ready_since = time.monotonic()
            if time.monotonic() - self.recovery_ready_since >= \
                    self.config.settings.recovery_stable_wall_s:
                self.engine.resume(now)
                self.transport_warning = ''
                self.recovery_gate = ''
                self.resume_after_sequence = -1
            return
        self.recovery_ready_since = None

    def _collision_tick(self, now):
        if self.contact_feed:
            if self.contact_feed.overflow:
                self.collision.error = 'CONTACT_FRAME_QUEUE_OVERFLOW'
            for frame in self.contact_feed.drain():
                self.collision.ingest(*frame)
        if self.journal:
            while self.collision.events:
                contact = self.collision.events[0]
                event = dict(contact, type='COLLISION', target_id=(
                    self.engine.target.target_id if self.engine.target else ''),
                    task_id=self.engine.task_id, robot_ids=list(self.config.robots),
                    dwell_elapsed_s=0.0, reason='PHYSICAL_CONTACT')
                saved = self.journal.event(event)
                self.collision.events.pop(0)
                self._publish_event(saved)
                self.get_logger().error('PHYSICAL CONTACT: ' + str(contact['pair']))
            if (self.collision.count and self.safety_future is None
                    and self.phase.phase == CompetitionState.PHASE_TASK2A
                    and self.phase_client.service_is_ready()):
                self.safety_future = self.phase_client.call_async(SetCompetitionPhase.Request(
                    run_id=self.engine.run_id, requested_phase=CompetitionState.PHASE_FROZEN,
                    requester_type=SetCompetitionPhase.Request.REQUESTER_SYSTEM,
                    requester_node=self.get_fully_qualified_name(), reason='physical contact'))
        snapshot = self.collision.snapshot(now, time.monotonic())
        self.collision_pub.publish(String(data=json.dumps(snapshot, allow_nan=False)))

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _request_finish(self):
        if not self.get_parameter('request_finish').value:
            self.finish_request = 'DISABLED'
        elif self.phase.phase != CompetitionState.PHASE_TASK2A:
            self.finish_request = 'PHASE_ALREADY_ENDED'
        elif self.finish_future is None and self.phase_client.service_is_ready():
            request = SetCompetitionPhase.Request(
                run_id=self.engine.run_id,
                requested_phase=SetCompetitionPhase.Request.PHASE_FROZEN,
                requester_type=SetCompetitionPhase.Request.REQUESTER_SYSTEM,
                requester_node=self.get_fully_qualified_name(),
                reason='official midterm S1 and C1 completed and robot actions released')
            self.finish_request = 'PENDING'
            self.finish_future = self.phase_client.call_async(request)
        elif self.finish_future is None:
            self.finish_request = 'WAITING_FOR_P2_SERVICE'
        if self.finish_future is not None and self.finish_future.done():
            try:
                response = self.finish_future.result()
                self.finish_request = ('ACCEPTED' if response.accepted
                                       else 'REJECTED:' + response.message)
            except Exception as error:
                self.finish_request = 'TRANSPORT_ERROR:' + str(error)

    def _publish_event(self, event):
        types = {'ASSIGNED': RescueEvent.EVENT_ASSIGNED,
                 'NAVIGATION_STARTED': RescueEvent.EVENT_NAVIGATION_STARTED,
                 'DWELL_STARTED': RescueEvent.EVENT_DWELL_STARTED,
                 'DWELL_RESET': RescueEvent.EVENT_DWELL_RESET,
                 'COMPLETED': RescueEvent.EVENT_COMPLETED,
                 'FAILED': RescueEvent.EVENT_FAILED,
                 'HALTED': RescueEvent.EVENT_BLOCKED}
        types['COLLISION'] = RescueEvent.EVENT_BLOCKED
        if event['type'] not in types:
            return
        msg = RescueEvent(run_id=self.engine.run_id, event_id=event['event_id'],
                          task_id=event['task_id'], target_id=event['target_id'],
                          scoring_stage=RescueEvent.STAGE_TASK2A, event_type=types[event['type']],
                          robot_ids=event['robot_ids'])
        msg.header.frame_id = 'map'
        msg.header.stamp.sec = int(event['sim_time'])
        msg.header.stamp.nanosec = int((event['sim_time'] % 1) * 1e9)
        msg.rescue_zone_indices = list(range(len(msg.robot_ids)))
        msg.robot_poses = []
        for rid in msg.robot_ids:
            robot = self.robots.get(rid)
            raw = self.raw_states.get(rid)
            pose = copy.deepcopy(raw.pose.pose) if raw else Pose()
            if robot:
                pose.position.x, pose.position.y = robot.x, robot.y
            msg.robot_poses.append(pose)
        elapsed = event['dwell_elapsed_s']
        msg.dwell_elapsed.sec = int(elapsed)
        msg.dwell_elapsed.nanosec = int(elapsed % 1 * 1e9)
        reason = event['reason']
        codes = {'': TaskReason.NONE, 'ROBOT_STATE_INVALID': TaskReason.ROBOT_UNAVAILABLE,
                 'ACTION_REJECTED': TaskReason.ROBOT_UNAVAILABLE,
                 'ACTION_RESPONSE_TIMEOUT': TaskReason.NAVIGATION_TIMEOUT,
                 'NAVIGATION_TIMEOUT': TaskReason.NAVIGATION_TIMEOUT,
                 'TASK_TIMEOUT': TaskReason.PARTNER_UNAVAILABLE,
                 'OUTSIDE_OR_INVALID_SAMPLE': TaskReason.DWELL_RESET,
                 'PHASE_OR_SAFETY_STOP': TaskReason.SAFETY_STOP if self.estop
                 else TaskReason.PHASE_CHANGED,
                 'RELEASE_NOT_CONFIRMED': TaskReason.SAFETY_STOP,
                 'CLOCK_REWIND': TaskReason.CONFIGURATION_INVALID}
        codes['PHYSICAL_CONTACT'] = TaskReason.SAFETY_STOP
        code = codes.get(reason, TaskReason.INTERNAL_ERROR)
        if reason.startswith('ACTION_ENDED:'):
            code = int(reason.split(':', 2)[1])
        msg.reason = TaskReason(code=code, detail=reason)
        self.event_pub.publish(msg)

    def _flush(self):
        while self.engine.events:
            event = self.engine.events[0]
            event['robots'] = {r: asdict(self.robots[r]) for r in event['robot_ids']
                               if r in self.robots}
            target = next((t for t in self.config.targets
                           if t.target_id == event['target_id']), None)
            if target:
                event['verification_region'] = {'kind': 'target_center_circle',
                                                'center': list(target.center),
                                                'radius_m': target.radius,
                                                'required_dwell_s': target.dwell}
            saved = self.journal.event(event)
            self.engine.events.pop(0)
            if saved:
                self._publish_event(saved)
                self.get_logger().info(
                    f'{event["type"]} {event["target_id"]} {event["reason"]}')
        self._save()

    def _save(self):
        if self.journal:
            self.journal.atomic('snapshot.json',
                                dict(self.engine.snapshot(), goals=self.goal_records,
                                     collision_history=self.collision.history(),
                                     resume_after_sequence=self.resume_after_sequence,
                                     finish_request=self.finish_request))

    def _tick(self):
        try:
            self._advance()
        except Exception as error:
            message = f'FAIL_CLOSED:{type(error).__name__}:{error}'
            if message != self.last_error:
                self.get_logger().error(message)
            self.last_error = message
            if self.engine:
                self.engine.status = 'HALTED'
                self.engine.failure = message
            # Cancellation must still run when storage or serialization fails.
            for token in list(self.goal_records):
                self.cancel_pending.add(token)
                self._cancel(token)
            self.status_pub.publish(String(data=json.dumps({'status': 'HALTED', 'error': message})))

    def _advance(self):
        now = self._now()
        if self.pending_map is not None and self._rescue_phase():
            pending, self.pending_map = self.pending_map, None
            self._on_map(pending)
        if self.map_future is not None and self.map_future.done():
            self.grid = self.map_future.result()
            self.map_future = None
        if self.last_ros_time is not None and now < self.last_ros_time:
            self.robots.clear()
            self.raw_states.clear()
        self.last_ros_time = now
        if self.engine is None and now > 0 and self._rescue_phase() \
                and self.phase.run_id and self.phase.map_id \
                and self.phase.map_revision > 0:
            self._start()
        self._collision_tick(now)
        enabled = self._enabled(now)
        if self.engine:
            if (self.phase.run_id, self.phase.map_id, self.phase.map_revision) != (
                    self.identity['run_id'], self.identity['map_id'],
                    self.identity['map_revision']):
                self.last_error = 'RUN_OR_MAP_CHANGED_RESTART_REQUIRED'
                enabled = False
            if (self.phase.phase != CompetitionState.PHASE_TASK2A
                    or self.phase.frozen or self.phase.estop_active or self.estop):
                self.resume_after_sequence = max(self.resume_after_sequence,
                                                 self.phase.phase_sequence)
            self.engine.tick(now, self.robots, enabled,
                             self.grid if self.grid is not None else math.dist)
            if self.engine.failure == 'ACTION_RESPONSE_TIMEOUT':
                for token in self.goal_records:
                    if token not in self.handles:
                        self.orphaned.add(token)
                        self.cancel_pending.add(token)
            self._flush()
            if self.engine.status == 'ACTIVE' and enabled:
                for token, slot in self.engine.slots.items():
                    robot = self.robots.get(slot['robot'])
                    if slot['state'] == 'transit_ready' and robot and robot.idle(
                            now, self.config.settings.state_stale_s) and self.engine.can_enter(
                                slot['index'], self.robots):
                        self.engine.rescue_requested(token, now)
                        self._send_rescue(Command('send', slot['robot'], token, slot['index']))
            for command in self.engine.drain_commands():
                if command.kind == 'send' and enabled:
                    self._send(command)
                elif command.kind == 'cancel':
                    self.cancel_pending.add(command.token)
            self._flush()
            for token in list(self.cancel_pending):
                self._cancel(token)
            self._try_recover(now, enabled)
            self._save()
            if self.engine.status == 'DONE' and not self.goal_records:
                self._request_finish()
            snapshot = dict(self.engine.snapshot(), sim_time=now, schema_version=1,
                            rule_profile=self.config.profile, identity=self.identity,
                            error=self.last_error, action_goals_outstanding=len(self.goal_records),
                            transport_warning=self.transport_warning,
                            finish_request=self.finish_request,
                            coordinator_instance=self.instance_id,
                            recovery_gate=self.recovery_gate,
                            collision_status=self.collision.snapshot(
                                now, time.monotonic())['state'],
                            collision=self.collision.snapshot(now, time.monotonic()))
            sample_key = (now, self.engine.task_id)
            if sample_key != self.last_sample:
                self.journal.sample(snapshot)
                self.last_sample = sample_key
        else:
            snapshot = {'schema_version': 1, 'status': 'WAITING_FOR_PHASE_MAP_AND_CLOCK',
                        'error': self.last_error}
        self.status_pub.publish(String(data=json.dumps(snapshot, allow_nan=False)))
        aggregate = RobotStateArray(robots=list(self.raw_states.values()))
        aggregate.header.frame_id = 'map'
        aggregate.header.stamp = self.get_clock().now().to_msg()
        aggregate.run_id = self.phase.run_id if self.phase else ''
        self.aggregate_pub.publish(aggregate)

    def close(self):
        """Best-effort cancellation; the existing P1 watchdog remains authoritative."""
        for token in list(self.goal_records):
            self._cancel(token)
        if self.journal:
            self.journal.close()
        if self.contact_feed:
            self.contact_feed.close()
        self.map_worker.shutdown(wait=False, cancel_futures=True)
        self.registry.destroy_node()
        self.verifier_node.destroy_node()
        self.destroy_node()


def main(args=None):
    # Keep the context alive during Ctrl+C so cancellation can reach P1 before
    # destroying the clients; P2 may still be publishing its phase heartbeat.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Coordinator()
    executor = SingleThreadedExecutor()
    for item in (node, node.registry, node.verifier_node):
        executor.add_node(item)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        deadline = time.monotonic() + 3.0
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if not node.goal_records:
                    break
        except KeyboardInterrupt:
            pass
        node.timer.cancel()
        node.close()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
