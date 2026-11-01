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

"""Authoritative competition state, safe handover, reset and Task 1 finalization."""

import json
import threading
import time
import uuid

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from edgeai_common.config import find_config_dir, load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_exploration.frontier import Grid
from edgeai_exploration.ros_grid import from_message
from edgeai_robot_interfaces.msg import CompetitionState, EmergencyStop, RobotPhaseAck
from edgeai_robot_interfaces.msg import RobotState, TargetArray
from edgeai_robot_interfaces.srv import FinalizeTask1, Relocalize, RequestReset, SetCompetitionPhase
from jsonschema.exceptions import ValidationError
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .bootstrap import initialize_artifacts, targets_to_document
from .freeze import Task1Transaction
from .phases import all_safe, MOTION_PHASES, PhasePolicy, PHASES, PhaseState
from .storage import RunStore, sha256, utc_now


class SystemManager(Node):
    def __init__(self, **kwargs):
        super().__init__('system_manager', namespace='/edge', **kwargs)
        for name, value in [('run_id', ''), ('runs_root', 'runs'), ('config_dir', ''),
                            ('run_mode', 'development'), ('robot_ids', ['robot_1']),
                            ('heartbeat_hz', 5.0), ('start_pose_ids', '')]:
            self.declare_parameter(name, value)
        # 'robot_2:midterm_robot_2,...': relocalize these robots at a known start pose
        # (a parked robot in the start area) instead of global localization.
        self.start_pose_ids = dict(
            item.split(':', 1) for item in self.get_parameter('start_pose_ids').value.split(',')
            if item)
        self.config = find_config_dir(self.get_parameter('config_dir').value)
        self.contract = load_runtime_contract(self.config)
        self.mode = self.get_parameter('run_mode').value
        self.robots = list(self.get_parameter('robot_ids').value)
        self.run_id = self.get_parameter('run_id').value
        self.store = RunStore(self.get_parameter('runs_root').value, self.run_id,
                              {'run_mode': self.mode, 'robots': self.robots})
        self.resolved, self.schemas = initialize_artifacts(
            self.store, self.config, self.mode, self.robots)
        saved = self.store.read('system/state.json')
        self.policy = PhasePolicy(self.contract, PhaseState(**saved['policy']) if saved else None)
        self.phase_started_ns = saved['phase_started_ns'] if saved else self.now_ns()
        self.reason = 'startup'
        self.task1_end_reason = saved.get('task1_end_reason', '') if saved else ''
        self.state_guard = threading.RLock()
        self.control_events = saved.get('control_events', []) if saved else []
        self.elapsed_by_phase = saved.get('elapsed_by_phase', {}) if saved else {}
        self.reset_started_ns = saved.get('reset_started_ns') if saved else None
        self.reset_id = saved.get('reset_id', '') if saved else ''
        self.transaction = Task1Transaction(self.store, self.schemas)
        self.transaction.recover()
        self.operation = threading.Lock()
        self.acks, self.robot_states = {}, {}
        self.metadata, self.targets, self.grid = None, None, None
        self.logger_status = None
        self.handover_ready = False
        self.handover_pose = None
        self.group = ReentrantCallbackGroup()
        control = qos_from_contract(self.contract, 'control_state')
        snapshot = qos_from_contract(self.contract, 'snapshot')
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(String, '/edge/logger_status', self.on_logger, snapshot,
                                 callback_group=self.group)
        self.state_pub = self.create_publisher(CompetitionState, '/competition/state', control)
        self.estop_pub = self.create_publisher(EmergencyStop, '/competition/estop', control)
        self.create_subscription(String, '/edge/map_metadata', self.on_metadata, snapshot,
                                 callback_group=self.group)
        self.create_subscription(TargetArray, '/edge/targets', self.on_targets, snapshot,
                                 callback_group=self.group)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, snapshot,
                                 callback_group=self.group)
        self.relocalizers = {}
        for robot in self.robots:
            self.create_subscription(RobotState, f'/{robot}/state', self.on_robot,
                                     qos_from_contract(self.contract, 'robot_state'),
                                     callback_group=self.group)
            self.create_subscription(RobotPhaseAck, f'/{robot}/phase_ack', self.on_ack,
                                     qos_from_contract(self.contract, 'robot_state'),
                                     callback_group=self.group)
            self.relocalizers[robot] = self.create_client(
                Relocalize, f'/{robot}/relocalize', callback_group=self.group)
        self.seal_client = self.create_client(
            Trigger, '/edge/event_logger/seal', callback_group=self.group)
        self.freeze_client = self.create_client(
            Trigger, '/edge/map_manager/freeze', callback_group=self.group)
        for srv, name, callback in [
                (SetCompetitionPhase, '/competition/set_phase', self.set_phase),
                (FinalizeTask1, '/edge/finalize_task1', self.finalize),
                (RequestReset, '/competition/request_reset', self.reset)]:
            self.create_service(srv, name, callback, callback_group=self.group)
        if saved and self.policy.state.phase in MOTION_PHASES | {2}:
            self.policy.transition(7, 'system')
            self.reason = 'restart_requires_operator_recovery'
        self.save()
        rate = max(self.contract['rates_hz']['competition_state_heartbeat'],
                   self.get_parameter('heartbeat_hz').value)
        self.create_timer(1 / rate, self.publish, callback_group=self.group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(0.5, self.watch, callback_group=self.group)
        self.publish()

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def save(self):
        self.store.write('system/state.json', {'policy': self.policy.state.snapshot(),
                                               'phase_started_ns': self.phase_started_ns,
                                               'reset_id': self.reset_id,
                                               'control_events': self.control_events,
                                               'elapsed_by_phase': self.elapsed_by_phase,
                                               'reset_started_ns': self.reset_started_ns,
                                               'task1_end_reason': self.task1_end_reason})

    def on_logger(self, message):
        try:
            value = json.loads(message.data)
            if isinstance(value, dict) and value.get('run_id') == self.run_id:
                self.logger_status = value, time.monotonic()
        except (ValueError, TypeError):
            self.get_logger().error('invalid logger status')

    def on_metadata(self, message):
        try:
            value = json.loads(message.data)
            if value['run_id'] == self.run_id:
                with self.state_guard:
                    previous = self.metadata[0] if self.metadata else None
                    self.metadata = (value, time.monotonic())
                    # The first identity counts as a change too: messages before it had
                    # no map_id, and one phase_sequence must never carry two identities.
                    if previous is None or (previous['map_id'], previous['revision']) != (
                            value['map_id'], value['revision']):
                        # A new identity must be acknowledged before any consumer
                        # receives goals/relocalization requests for that identity.
                        self.policy.state.sequence += 1
                        self.control_event('map', 'version_changed', {
                            'map_id': value['map_id'], 'map_revision': value['revision'],
                            'phase_sequence': self.policy.state.sequence})
                        self.save()
                self.publish()
        except (ValueError, KeyError):
            self.get_logger().error('invalid map metadata')

    def on_targets(self, message):
        if message.run_id == self.run_id and message.header.frame_id == 'map':
            self.targets = message

    def on_map(self, message):
        try:
            self.grid = from_message(message)
        except ValueError as error:
            self.get_logger().error(str(error))

    def on_robot(self, message):
        if message.run_id == self.run_id and message.robot_id in self.robots:
            self.robot_states[message.robot_id] = (message, time.monotonic())

    def on_ack(self, message):
        if message.run_id == self.run_id and message.robot_id in self.robots:
            self.acks[message.robot_id] = (message, time.monotonic())

    def map_metadata(self):
        if self.metadata is None or time.monotonic() - self.metadata[1] > 3:
            raise ValueError('map manager unavailable')
        meta = self.metadata[0]
        if not meta.get('has_map'):
            raise ValueError('map snapshot unavailable')
        if not meta.get('frozen'):
            if not meta.get('slam_running'):
                raise ValueError('SLAM process unavailable')
            if meta.get('map_age_s') is not None and meta['map_age_s'] > 10:
                raise ValueError('SLAM map updates stale')
        return meta

    def publish(self):
        with self.state_guard:
            # Phase and map identity from one consistent snapshot: on_metadata bumps the
            # sequence together with the identity (P4 rejects a sequence that changes).
            state = PhaseState(**self.policy.state.snapshot())
            meta = self.metadata[0] if self.metadata else None
        message = CompetitionState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.run_id = self.run_id
        message.run_mode = ('development', 'rehearsal', 'competition').index(self.mode)
        message.parameters_sha256 = self.resolved.sha256
        message.phase, message.phase_sequence = state.phase, state.sequence
        message.interrupted_phase = state.interrupted
        message.phase_started_at.sec = self.phase_started_ns // 1000000000
        message.phase_started_at.nanosec = self.phase_started_ns % 1000000000
        message.frozen = state.phase in (2, 4, 6, 7)
        message.estop_active = state.phase == 7
        message.reset_count = state.reset_count
        message.reset_in_progress = bool(state.resetting_robot_ids)
        message.resetting_robot_ids = state.resetting_robot_ids
        message.reason = self.reason
        if meta:
            message.map_id, message.map_revision = meta['map_id'], meta['revision']
        if self.targets:
            message.target_revision = self.targets.revision
        self.state_pub.publish(message)
        stop = EmergencyStop()
        stop.header = message.header
        stop.run_id, stop.generation = self.run_id, state.generation
        stop.active = message.estop_active
        stop.source, stop.reason = '/edge/system_manager', self.reason
        self.estop_pub.publish(stop)
        diagnostics = DiagnosticArray()
        diagnostics.header = message.header
        status = DiagnosticStatus()
        status.name, status.hardware_id = 'edge/system_manager', self.run_id
        status.level = DiagnosticStatus.ERROR if state.phase == 7 else DiagnosticStatus.OK
        status.message = self.reason
        if self.resolved.missing_for_motion and self.mode == 'competition':
            status.level = DiagnosticStatus.ERROR
            status.message = 'CODE_OFFICIAL_PARAMETER_MISSING'
        status.values = [KeyValue(key='phase', value=PHASES[state.phase]),
                         KeyValue(key='phase_sequence', value=str(state.sequence)),
                         KeyValue(key='missing_for_motion',
                                  value=','.join(self.resolved.missing_for_motion))]
        diagnostics.status = [status]
        self.diagnostics_pub.publish(diagnostics)

    def control_event(self, category, event_type, payload, event_id=None):
        phase = PHASES[self.policy.state.phase].lower()
        self.control_events.append({
            'event_id': event_id or 'control-' + uuid.uuid4().hex,
            'ros_time_ns': self.now_ns(), 'source_node': '/edge/system_manager',
            'phase': phase,
            'scoring_stage': phase if phase in ('task1', 'task2a', 'task2b') else 'none',
            'category': category, 'event_type': event_type, 'payload': payload})

    def change(self, destination, requester, reason):
        with self.state_guard:
            previous = self.policy.state.phase
            if self.policy.transition(destination, requester):
                now = self.now_ns()
                if previous in MOTION_PHASES:
                    key = str(previous)
                    self.elapsed_by_phase[key] = self.elapsed_by_phase.get(key, 0) + max(
                        0, now - self.phase_started_ns)
                self.phase_started_ns = now
                self.reason = reason
                if destination == 2:
                    self.task1_end_reason = reason
                self.control_event('phase', 'transition', {
                    'from_phase': PHASES[previous].lower(),
                    'to_phase': PHASES[destination].lower(),
                    'phase_sequence': self.policy.state.sequence,
                    'requester_type': requester, 'reason': reason},
                    event_id=f'phase:{self.policy.state.sequence}')
                self.save()
        self.publish()

    def wait(self, predicate, seconds, description):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            if predicate():
                return
            time.sleep(0.02)
        raise TimeoutError(description)

    def safe(self):
        state = self.policy.state
        return all_safe(self.robots, self.acks, self.run_id, state.sequence, state.phase,
                        time.monotonic(), self.contract['timeouts_s']['robot_state_stale'])

    def call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=min(2.0, timeout)):
            raise TimeoutError('service unavailable: ' + client.srv_name)
        future = client.call_async(request)
        self.wait(future.done, timeout, 'service timeout: ' + client.srv_name)
        return future.result()

    def logger_ready(self):
        return (self.logger_status is not None
                and time.monotonic() - self.logger_status[1] <= 3
                and self.logger_status[0]['ready'])

    def ready_to_move(self, check_localization=True):
        if self.resolved.missing_for_motion and self.mode != 'development':
            raise ValueError('CODE_OFFICIAL_PARAMETER_MISSING: ' + ', '.join(
                self.resolved.missing_for_motion))
        if self.policy.state.resetting_robot_ids:
            raise ValueError('reset is in progress')
        if not self.logger_ready():
            raise ValueError('event logger is not ready')
        self.map_metadata()
        now = time.monotonic()
        for robot in self.robots:
            entry = self.robot_states.get(robot)
            if not entry or now - entry[1] > self.contract['timeouts_s']['robot_state_stale']:
                raise ValueError(robot + ' state missing/stale')
        primary = self.robot_states[self.robots[0]][0]
        if check_localization and (not primary.localized or not primary.navigation_available):
            raise ValueError('primary robot localization/navigation not ready')

    def set_phase(self, request, response):
        # Emergency stop bypasses long-running freeze/relocalization operations.
        if (request.run_id == self.run_id and request.requested_phase == 7
                and request.requester_type in (1, 2)):
            self.change(7, 'operator' if request.requester_type == 1 else 'system', request.reason)
            response.accepted = True
            response.current_phase = 7
            response.phase_sequence = self.policy.state.sequence
            response.message = 'ESTOP latched; safe acknowledgement monitored separately'
            return response
        if not self.operation.acquire(blocking=False):
            response.message = 'another control operation is active'
            response.current_phase = self.policy.state.phase
            response.phase_sequence = self.policy.state.sequence
            return response
        try:
            if request.run_id != self.run_id:
                raise ValueError('run_id mismatch')
            requester = {1: 'operator', 2: 'system'}.get(request.requester_type, '')
            destination = self.policy.destination(request.requested_phase, requester)
            if destination in MOTION_PHASES:
                self.ready_to_move(check_localization=destination not in (3, 5))
            if destination in (3, 5):
                if not self.store.read('manifest.json')['task1']:
                    raise ValueError('Task 1 is not finalized')
                if not self.handover_ready or destination == 5:
                    self.relocalize(self.robots, handover=destination == 3)
            if destination == 2 and self.policy.state.phase == 1:
                self.handover_pose = self.robot_states.get('robot_1', (None,))[0]
            self.change(destination, requester, request.reason)
            if destination in (2, 4, 6, 7):
                self.wait(self.safe, self.contract['timeouts_s']['phase_ack'],
                          'safe phase acknowledgements missing')
            if destination == 2:
                result = self.call(self.freeze_client, Trigger.Request(), 20.0)
                if not result.success:
                    raise ValueError(result.message)
                self.wait(lambda: self.map_metadata()['frozen'], 3, 'map freeze not confirmed')
                self.wait(self.safe, 3, 'frozen map identity not acknowledged')
            if destination == 6:
                result = self.call(self.seal_client, Trigger.Request(), 25)
                if not result.success:
                    raise ValueError(result.message)
                manifest = self.store.read('manifest.json')
                self.transaction.verify(manifest)
                files = {entry['path']: entry for entry in manifest['files']}
                for entry in json.loads(result.message):
                    if entry['path'] in files and files[entry['path']] != entry:
                        raise ValueError('immutable manifest entry changed')
                    files[entry['path']] = entry
                if self.store.read('system/final_state.json') is None:
                    self.store.write('system/final_state.json',
                                     self.store.read('system/state.json'))
                for relative in ('system/final_state.json', 'system/sealed.json',
                                 'system/identity.json'):
                    path = self.store.path(relative)
                    files[relative] = {'path': relative, 'role': 'other', 'stage': 'system',
                                       'sha256': sha256(path), 'size_bytes': path.stat().st_size}
                manifest['files'] = list(files.values())
                manifest['resets'] = self.policy.state.reset_count
                manifest['manifest_revision'] += 1
                manifest['updated_at'] = utc_now()
                self.transaction.verify(manifest)
                self.store.write('manifest.json', manifest)
            response.accepted = True
            response.message = 'phase confirmed'
        except (ValueError, TimeoutError, OSError) as error:
            response.message = str(error)
            # A failed safe transition stays safe; do not roll back into motion.
            self.reason = str(error)
        finally:
            response.current_phase = self.policy.state.phase
            response.phase_sequence = self.policy.state.sequence
            self.operation.release()
        return response

    def relocalize(self, robots, handover=False):
        meta = self.map_metadata()
        if not meta['frozen'] or not self.safe():
            raise ValueError('relocalization requires frozen map and safe acknowledgements')
        for robot in robots:
            request = Relocalize.Request()
            request.run_id, request.map_id = self.run_id, meta['map_id']
            request.map_revision = meta['revision']
            request.mode = Relocalize.Request.MODE_GLOBAL
            if robot in self.start_pose_ids:
                request.mode = Relocalize.Request.MODE_KNOWN_START_POSE
                request.start_pose_id = self.start_pose_ids[robot]
            if handover and robot == 'robot_1' and self.handover_pose is not None:
                request.mode = Relocalize.Request.MODE_POSE_HINT
                request.pose_hint.header.frame_id = 'map'
                request.pose_hint.header.stamp = self.get_clock().now().to_msg()
                request.pose_hint.pose.pose = self.handover_pose.pose.pose
            timeout = self.contract['timeouts_s']['relocalize_default']
            request.timeout.sec = int(timeout)
            result = self.call(self.relocalizers[robot], request, timeout + 3)
            if not result.success:
                raise ValueError(robot + ' relocalization failed: ' + result.message)
        self.handover_ready = True

    def finalize(self, request, response):
        if not self.operation.acquire(blocking=False):
            response.message = 'another control operation is active'
            return response
        try:
            if request.run_id != self.run_id or self.policy.state.phase != 2 or not self.safe():
                raise ValueError('finalization requires matching run and confirmed HANDOVER')
            meta = self.map_metadata()
            if (not meta['frozen'] or meta['map_id'] != request.map_id
                    or meta['revision'] != request.expected_map_revision):
                raise ValueError('map version is not frozen/matching')
            targets = self.targets
            if (targets is None or not targets.task1_frozen
                    or targets.map_id != meta['map_id'] or targets.map_revision != meta['revision']
                    or targets.revision != request.expected_target_revision):
                raise ValueError('awaiting P3 frozen target snapshot with matching versions')
            geometry = self.resolved.get('rescue.geometry_version')
            source = ('development' if 'rescue.geometry_version' in self.resolved.fallbacks
                      else 'official') if geometry else None
            document = targets_to_document(targets, geometry, source)
            snapshot = self.store.read('system/map_snapshot.json')
            if (snapshot is None or not snapshot['frozen'] or snapshot['run_id'] != self.run_id
                    or snapshot['map_id'] != meta['map_id']
                    or snapshot['revision'] != meta['revision']):
                raise ValueError('durable frozen map snapshot missing or mismatched')
            frozen_grid = Grid(**snapshot['grid'])
            self.transaction.commit(frozen_grid, document, self.store.read('manifest.json'),
                                    meta['map_id'], meta['revision'], targets.revision,
                                    'time_limit' if self.task1_end_reason == 'time_limit'
                                    else 'team_declared_complete')
            response.success = True
            response.manifest_uri = self.store.path('manifest.json').as_uri()
            response.map_yaml_uri = self.store.path('task1/map.yaml').as_uri()
            response.target_list_uri = self.store.path('task1/targets.json').as_uri()
            response.message = 'Task 1 snapshot committed and verified'
        except (ValueError, ValidationError, OSError) as error:
            response.message = str(error)
        finally:
            self.operation.release()
        return response

    def reset(self, request, response):
        if not self.operation.acquire(blocking=False):
            response.message = 'another control operation is active'
            return response
        try:
            if request.run_id != self.run_id or not request.approved_by:
                raise ValueError('run mismatch or missing operator approval identity')
            if self.mode == 'competition':
                rules = self.resolved.get('reset', {})
                required = ('allowed_count', 'penalty', 'timing_continues',
                            'retain_map', 'retain_targets', 'retain_completed_rescues')
                if any(rules.get(k) is None for k in required):
                    raise ValueError('official reset rules incomplete')
                retained = ('retain_map', 'retain_targets', 'retain_completed_rescues')
                if not all(rules[k] for k in retained):
                    raise ValueError('non-retaining reset requires a new coordinated run')
            state = self.policy.state
            robots = sorted(request.robot_ids or self.robots)
            if not set(robots) <= set(self.robots):
                raise ValueError('unknown reset robot')
            if request.action == 1:
                if state.resetting_robot_ids:
                    if robots != state.resetting_robot_ids:
                        raise ValueError('different reset already in progress')
                else:
                    allowed = self.resolved.get('reset.allowed_count')
                    if allowed is not None and state.reset_count >= allowed:
                        raise ValueError('reset count limit exceeded')
                    self.change(7, 'operator', 'reset_begin: ' + request.reason)
                    self.wait(self.safe, 3.0, 'reset stop not confirmed')
                    state.reset_count += 1
                    self.reset_started_ns = self.now_ns()
                    state.resetting_robot_ids = robots
                    self.reset_id = 'reset-' + uuid.uuid4().hex
            elif request.action == 2:
                if not state.resetting_robot_ids or robots != state.resetting_robot_ids:
                    raise ValueError('no matching reset')
                if state.phase == 7:
                    self.change(4, 'operator', 'reset_relocalizing')
                    self.wait(self.safe, 3.0, 'reset phase not confirmed')
                self.relocalize(robots)
                state.resetting_robot_ids = []
            elif request.action == 3:
                if not state.resetting_robot_ids:
                    raise ValueError('no reset in progress')
                state.resetting_robot_ids = []
                # Abort deliberately leaves the deployment stopped.
            else:
                raise ValueError('unknown reset action')
            action_name = {1: 'begin', 2: 'complete', 3: 'abort'}[request.action]
            self.control_event('reset', 'reset_' + action_name, {
                'reset_id': self.reset_id, 'action': action_name, 'robot_ids': robots,
                'approved_by': request.approved_by, 'reset_count': state.reset_count})
            if request.action in (2, 3) and self.reset_started_ns is not None:
                if self.resolved.get('reset.timing_continues', True):
                    key = str(state.interrupted)
                    self.elapsed_by_phase[key] = self.elapsed_by_phase.get(key, 0) + max(
                        0, self.now_ns() - self.reset_started_ns)
                self.reset_started_ns = None
            self.save()
            self.publish()
            response.accepted = True
            response.message = 'reset operation complete; motion requires operator phase request'
        except (ValueError, TimeoutError, OSError) as error:
            response.message = str(error)
        finally:
            response.reset_id = self.reset_id
            response.reset_count = self.policy.state.reset_count
            self.operation.release()
        return response

    def watch(self):
        state = self.policy.state
        if state.phase not in MOTION_PHASES or self.operation.locked():
            return
        try:
            self.map_metadata()
            if not self.logger_ready():
                raise ValueError('event logger failed or disconnected')
            now = time.monotonic()
            if any(robot not in self.robot_states or now - self.robot_states[robot][1] >
                   self.contract['timeouts_s']['robot_state_stale'] for robot in self.robots):
                raise ValueError('robot state stale during motion')
            if len(self.get_publishers_info_by_topic('/competition/state')) != 1:
                raise ValueError('multiple competition state authorities detected')
        except ValueError as error:
            self.change(7, 'system', str(error))
            return
        delta = self.now_ns() - self.phase_started_ns
        elapsed = (self.elapsed_by_phase.get(str(state.phase), 0) + delta) / 1e9
        limit = self.resolved.get('timing.' + PHASES[state.phase].lower() + '_s')
        if delta < 0:
            self.change(7, 'system', 'ROS clock moved backwards')
        elif limit and elapsed >= limit:
            # Route through the same transition path to preserve safe confirmation.
            request = SetCompetitionPhase.Request()
            request.run_id, request.requester_type = self.run_id, 2
            request.requested_phase = {1: 2, 3: 4, 5: 6}[state.phase]
            request.reason = 'time_limit'
            self.set_phase(request, SetCompetitionPhase.Response())

    def destroy_node(self):
        self.store.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=6)
    try:
        node = SystemManager()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
