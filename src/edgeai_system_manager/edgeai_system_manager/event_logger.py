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

"""Sole schema-checked event journal writer and Task 2B artifact collector."""

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import uuid

from edgeai_common.config import find_config_dir, load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState, EmergencyStop, MissionIntakeResult
from edgeai_robot_interfaces.msg import MissionPlan, RescueEvent, TargetArray
from jsonschema.exceptions import ValidationError
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .phases import PHASES
from .storage import atomic_bytes, atomic_json, EventJournal, Schemas, sha256


def timestamp(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def datetime_of(stamp):
    return datetime.fromtimestamp(timestamp(stamp) / 1e9, timezone.utc).isoformat()


def steps_of(message):
    if any(step.action_type != 1 for step in message.steps):
        raise ValueError('unsupported mission action')
    return [{'step_id': step.step_id, 'action': 'rescue', 'target_id': step.target_id,
             'priority': step.priority, 'predecessor_step_ids': list(step.predecessor_step_ids)}
            for step in message.steps]


def model_of(message):
    return {'name': message.model_name, 'version': message.model_version,
            'prompt_version': message.prompt_version}


class EventLogger(Node):
    def __init__(self, **kwargs):
        super().__init__('event_logger', namespace='/edge', **kwargs)
        self.declare_parameter('run_directory', '')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('record_bag', False)
        self.directory = Path(self.get_parameter('run_directory').value).resolve()
        self.run_id = self.directory.name
        config = find_config_dir(self.get_parameter('config_dir').value)
        runtime = load_runtime_contract(config)
        self.schemas = Schemas(config.parent / 'schemas')
        self.journal = EventJournal(self.directory, self.run_id, self.schemas)
        self.health_error = ''
        self.sealed = (self.directory / 'system/sealed.json').exists()
        self.recorder = None
        self.recorder_log = None
        self.phase = 'preparation'
        self.sequence = None
        self.target_types = {}
        self.intakes = {}
        self.plans = {}
        self.health_pub = self.create_publisher(
            String, '/edge/logger_status', qos_from_contract(runtime, 'snapshot'))
        self.create_timer(0.5, self.publish_health)
        self.create_subscription(CompetitionState, '/competition/state', self.state,
                                 qos_from_contract(runtime, 'control_state'))
        self.create_subscription(EmergencyStop, '/competition/estop', self.estop,
                                 qos_from_contract(runtime, 'control_state'))
        self.create_subscription(TargetArray, '/edge/targets', self.targets,
                                 qos_from_contract(runtime, 'snapshot'))
        self.create_subscription(RescueEvent, '/edge/rescue_events', self.rescue,
                                 qos_from_contract(runtime, 'event_stream'))
        self.create_subscription(MissionIntakeResult, '/edge/mission_intake', self.intake,
                                 qos_from_contract(runtime, 'event_stream'))
        self.create_subscription(MissionPlan, '/edge/mission_plan', self.plan,
                                 qos_from_contract(runtime, 'snapshot'))
        self.create_service(Trigger, '/edge/event_logger/seal', self.seal)
        self.create_timer(0.25, self.outbox)
        if self.get_parameter('record_bag').value and not self.sealed:
            self.start_recorder()
        # Replay durable source messages if the logger died between receipt and export.
        for path in sorted((self.directory / 'task2b').glob('*/plan_source.json')):
            message = MissionPlan()
            set_message_fields(message, json.loads(path.read_text()))
            self.plan(message)
        for path in sorted((self.directory / 'task2b').glob('*/intake_source.json')):
            message = MissionIntakeResult()
            set_message_fields(message, json.loads(path.read_text()))
            self.intake(message)

    def publish_health(self):
        if self.recorder is not None and self.recorder.poll() is not None:
            self.health_error = 'rosbag recorder exited unexpectedly'
        self.health_pub.publish(String(data=json.dumps({
            'run_id': self.run_id, 'ready': not self.health_error and not self.sealed,
            'sealed': self.sealed, 'error': self.health_error,
            'event_sequence': self.journal.sequence})))

    def emit(self, category, event_type, event_id, stamp, payload, source, stage=None):
        if self.sealed:
            return
        try:
            if event_id in self.journal.ids:
                # Duplicate source messages may arrive in a different phase. Keep
                # the original receiving phase, but still reject altered payloads.
                previous = self.journal.ids[event_id]
                phase, scoring = previous['phase'], previous['scoring_stage']
            else:
                phase = self.phase
                scoring = stage or (phase if phase in ('task1', 'task2a', 'task2b') else 'none')
            self.journal.append(event_id=event_id, ros_time_ns=timestamp(stamp),
                                source_node=source, phase=phase, scoring_stage=scoring,
                                category=category, event_type=event_type, payload=payload)
        except (ValueError, ValidationError, OSError) as error:
            self.health_error = 'evidence rejected: ' + str(error)
            self.get_logger().error(self.health_error)

    def state(self, message):
        if message.run_id != self.run_id or message.phase >= len(PHASES):
            return
        self.phase = PHASES[message.phase].lower()
        self.sequence = message.phase_sequence

    def outbox(self):
        if self.sealed:
            return
        path = self.directory / 'system/state.json'
        if not path.exists():
            return
        try:
            state = json.loads(path.read_text())
            for record in state.get('control_events', []):
                self.journal.append(**record)
        except (ValueError, ValidationError, OSError) as error:
            self.health_error = 'control event rejected: ' + str(error)
            self.get_logger().error(self.health_error)

    def estop(self, message):
        if message.run_id != self.run_id:
            return
        key = f'estop:{message.generation}'
        if key in self.journal.ids:
            return
        self.emit('estop', 'activated' if message.active else 'released', key,
                  message.header.stamp, {'active': message.active, 'generation': message.generation,
                                         'source': message.source, 'reason': message.reason},
                  '/edge/system_manager')

    def targets(self, message):
        if message.run_id == self.run_id:
            self.target_types = {target.target_id: target.target_type for target in message.targets}

    def rescue(self, message):
        if message.run_id != self.run_id:
            return
        target_type = self.target_types.get(message.target_id)
        if target_type not in (1, 2) or message.scoring_stage not in (1, 2):
            self.health_error = 'rescue event lacks authoritative target type/stage'
            self.get_logger().error(self.health_error)
            return
        if not (len(message.robot_ids) == len(message.rescue_zone_indices)
                == len(message.robot_poses)):
            self.health_error = 'rescue event arrays have inconsistent lengths'
            self.get_logger().error(self.health_error)
            return
        names = {getattr(RescueEvent, key): key.lower() for key in dir(RescueEvent)
                 if key.startswith('EVENT_')}
        payload = {'task_id': message.task_id, 'target_id': message.target_id,
                   'target_type': 'normal' if target_type == 1 else 'cooperative',
                   'mission_id': message.mission_id, 'step_id': message.step_id,
                   'robot_ids': list(message.robot_ids),
                   'rescue_zone_indices': list(message.rescue_zone_indices),
                   'robot_positions': [{'x': pose.position.x, 'y': pose.position.y}
                                       for pose in message.robot_poses],
                   'dwell_elapsed_s': timestamp(message.dwell_elapsed) / 1e9,
                   'reason_code': message.reason.code, 'reason_detail': message.reason.detail}
        self.emit('rescue', names.get(message.event_type, 'unknown'), message.event_id,
                  message.header.stamp, payload, '/edge/rescue_verifier',
                  'task2a' if message.scoring_stage == 1 else 'task2b')

    def mission_directory(self, mission_id):
        # Do not use untrusted natural-language identifiers as filesystem paths.
        key = hashlib.sha256(mission_id.encode()).hexdigest()[:24]
        path = self.directory / 'task2b' / key
        path.mkdir(parents=True, exist_ok=True)
        return path

    def immutable(self, path, document):
        if path.exists():
            if json.loads(path.read_text()) != document:
                raise ValueError('immutable artifact conflict: ' + str(path))
        else:
            atomic_json(path, document)

    def intake(self, message):
        if message.run_id != self.run_id or self.sealed:
            return
        try:
            directory = self.mission_directory(message.mission_id)
            self.immutable(directory / 'intake_source.json', message_to_ordereddict(message))
            self.intakes[message.mission_id] = message
            self.write_mission(message.mission_id)
        except (ValueError, ValidationError, OSError) as error:
            self.health_error = 'mission intake rejected: ' + str(error)
            self.get_logger().error(self.health_error)

    def plan(self, message):
        if message.run_id != self.run_id or not message.valid or self.sealed:
            return
        try:
            directory = self.mission_directory(message.mission_id)
            self.immutable(directory / 'plan_source.json', message_to_ordereddict(message))
            self.plans[message.mission_id] = message
            self.write_mission(message.mission_id)
        except (ValueError, ValidationError, OSError) as error:
            self.health_error = 'mission plan rejected: ' + str(error)
            self.get_logger().error(self.health_error)

    def write_mission(self, mission_id):
        intake = self.intakes.get(mission_id)
        if intake is None:
            return
        directory = self.mission_directory(mission_id)
        outcomes = {1: 'interpretation_failed', 2: 'rejected_by_validator', 3: 'accepted'}
        outcome = outcomes.get(intake.outcome)
        if outcome is None:
            raise ValueError('unknown intake outcome')
        reference = None
        if outcome == 'accepted':
            plan = self.plans.get(mission_id)
            if plan is None:
                return
            identity = ('run_id', 'mission_id', 'map_id', 'map_revision', 'target_revision')
            if (any(getattr(plan, key) != getattr(intake.candidate_plan, key)
                    for key in identity)
                    or steps_of(plan) != steps_of(intake.candidate_plan)
                    or plan.original_instruction != intake.original_instruction
                    or model_of(plan) != model_of(intake)):
                raise ValueError('validated plan differs from intake')
            plan_path = directory / 'mission_plan.json'
            document = {'schema_version': 2, 'run_id': self.run_id, 'mission_id': mission_id,
                        'map_id': plan.map_id, 'map_revision': plan.map_revision,
                        'target_revision': plan.target_revision,
                        'created_at': datetime_of(intake.received_at),
                        'validated_at': datetime_of(plan.header.stamp),
                        'original_instruction': plan.original_instruction,
                        'model': model_of(plan), 'steps': steps_of(plan)}
            self.schemas.check('mission_plan', document)
            self.immutable(plan_path, document)
            reference = {'path': str(plan_path.relative_to(self.directory)),
                         'sha256': sha256(plan_path)}
        issues = []
        for issue in intake.issues:
            names = {getattr(type(issue), key): key.removeprefix('CODE_').lower()
                     for key in dir(type(issue)) if key.startswith('CODE_')}
            issues.append({'code': issue.code, 'code_name': names.get(issue.code, 'unknown'),
                           'step_id': issue.step_id, 'target_id': issue.target_id,
                           'detail': issue.detail})
        document = {'schema_version': 1, 'run_id': self.run_id, 'mission_id': mission_id,
                    'received_at': datetime_of(intake.received_at),
                    'original_instruction': intake.original_instruction, 'model': model_of(intake),
                    'raw_model_output': intake.raw_model_output, 'outcome': outcome,
                    'candidate_steps': steps_of(intake.candidate_plan), 'issues': issues,
                    'validated_plan': reference}
        self.schemas.check('mission_intake', document)
        self.immutable(directory / 'mission_intake.json', document)
        instruction = directory / 'original_instruction.txt'
        raw = intake.original_instruction.encode()
        if instruction.exists() and instruction.read_bytes() != raw:
            raise ValueError('instruction changed')
        if not instruction.exists():
            atomic_bytes(instruction, raw)
        self.emit('mission', 'intake_' + outcome, 'mission:' + mission_id,
                  intake.header.stamp, {'mission_id': mission_id, 'outcome': outcome},
                  '/edge/mission_interpreter', 'task2b')

    def start_recorder(self):
        base = self.directory / 'system/rosbag'
        base.mkdir(parents=True, exist_ok=True)
        output = base / ('segment-' + uuid.uuid4().hex[:12])
        topics = ['/clock', '/tf', '/tf_static', '/map', '/competition/state',
                  '/competition/estop', '/edge/targets', '/edge/exploration_status',
                  '/edge/rescue_events', '/edge/mission_plan', '/edge/mission_intake']
        for robot in range(1, 5):
            topics += [f'/robot_{robot}/scan', f'/robot_{robot}/odom',
                       f'/robot_{robot}/state', f'/robot_{robot}/phase_ack']
        self.recorder_log = (base / (output.name + '.log')).open('ab')
        self.recorder = subprocess.Popen(
            ['ros2', 'bag', 'record', '--storage', 'mcap', '-o', str(output), *topics],
            stdout=self.recorder_log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop_recorder(self):
        if self.recorder is None:
            return
        if self.recorder.poll() is None:
            os.killpg(self.recorder.pid, signal.SIGINT)
            try:
                self.recorder.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(self.recorder.pid, signal.SIGKILL)
                self.recorder.wait(timeout=5)
                raise OSError('rosbag did not close cleanly; audit/reindex before export')
        if self.recorder.returncode not in (0, -signal.SIGINT):
            raise OSError('rosbag recorder failed; see system/rosbag logs')
        self.recorder = None
        self.recorder_log.close()
        self.recorder_log = None

    def seal(self, request, response):
        del request
        try:
            state = json.loads((self.directory / 'system/state.json').read_text())
            if state['policy']['phase'] != 6:
                raise ValueError('event log sealing requires FINISHED')
            self.phase = 'finished'
            receipt = self.directory / 'system/sealed.json'
            if not self.sealed:
                self.outbox()
                if self.health_error:
                    raise ValueError(self.health_error)
                self.stop_recorder()
                files = []
                paths = [self.directory / 'events.jsonl']
                paths += sorted((self.directory / 'task2b').rglob('*'))
                paths += sorted((self.directory / 'system/rosbag').rglob('*'))
                for path in paths:
                    if not path.is_file():
                        continue
                    relative = str(path.relative_to(self.directory))
                    role = 'events' if path.name == 'events.jsonl' else {
                        'mission_plan.json': 'mission_plan',
                        'mission_intake.json': 'mission_intake',
                        'original_instruction.txt': 'original_instruction'}.get(path.name, 'other')
                    if relative.startswith('system/rosbag/'):
                        role = 'rosbag'
                    files.append({'path': relative, 'role': role,
                                  'stage': 'task2b' if relative.startswith('task2b/') else 'system',
                                  'sha256': sha256(path), 'size_bytes': path.stat().st_size})
                atomic_json(receipt, files)
                self.sealed = True
            files = json.loads(receipt.read_text())
            for entry in files:
                if sha256(self.directory / entry['path']) != entry['sha256']:
                    raise ValueError('sealed evidence changed')
            response.success = True
            response.message = json.dumps(files)
        except (ValueError, OSError) as error:
            response.message = str(error)
        return response

    def destroy_node(self):
        try:
            self.stop_recorder()
        finally:
            self.journal.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EventLogger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
