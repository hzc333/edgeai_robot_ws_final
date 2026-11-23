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

"""Run both rescue stages on an existing generated Gazebo practice arena."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from edgeai_common.config import load_runtime_contract, resolve_parameters
from edgeai_common.qos import qos_from_contract
from edgeai_rescue_coordinator.ros_support import to_dict
from edgeai_robot_interfaces.msg import (
    CompetitionState, MissionExecutionState, MissionIntakeResult, RescueEvent,
    RobotState, TargetArray,
)
from edgeai_robot_interfaces.srv import FinalizeTask1, SetCompetitionPhase, SubmitMission
from edgeai_system_manager.audit import audit
from edgeai_system_manager.storage import atomic_json, new_run_id
import rclpy
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[3]


class Rehearsal:
    """Use the existing operator services and collect results without directing robots."""

    def __init__(self, options):
        self.options = options
        self.robots = [f'robot_{i+1}' for i in range(options.robots)]
        self.run_id = new_run_id()
        self.directory = options.output.resolve() / self.run_id
        self.directory.parent.mkdir(parents=True, exist_ok=True)
        self.processes = []
        self.samples = {}
        self.events = []
        self.intakes = {}
        self.missing_targets = []
        self.failed_task2a_targets = []
        self.contract = load_runtime_contract()
        os.environ['GZ_PARTITION'] = 'edgeai-' + self.run_id
        os.environ['ROS_DOMAIN_ID'] = str(options.domain)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        os.environ.setdefault('ROS_LOG_DIR', str(self.directory.parent / 'ros'))
        os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                              '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                              '</Discovery></Domain></CycloneDDS>')
        rclpy.init()
        self.node = rclpy.create_node('frozen_rescue_rehearsal')
        for cls, topic, key, qos in (
            (CompetitionState, '/competition/state', 'phase', 'control_state'),
            (TargetArray, '/edge/targets', 'targets', 'snapshot'),
            (String, '/edge/map_metadata', 'map', 'snapshot'),
            (String, '/edge/rescue/status', 'rescue', 'snapshot'),
            (MissionExecutionState, '/edge/mission_state', 'mission', 'snapshot'),
            *((RobotState, f'/{r}/state', r, 'robot_state') for r in self.robots),
        ):
            self.node.create_subscription(
                cls, topic, lambda msg, key=key: self.samples.update({key: msg}),
                qos_from_contract(self.contract, qos))
        stream = qos_from_contract(self.contract, 'event_stream')
        self.node.create_subscription(
            RescueEvent, '/edge/rescue_events', self.events.append, stream)
        self.node.create_subscription(
            MissionIntakeResult, '/edge/mission_intake',
            lambda msg: self.intakes.update({msg.mission_id: msg}), stream)
        self.phase_client = self.node.create_client(SetCompetitionPhase, '/competition/set_phase')
        self.finalize = self.node.create_client(FinalizeTask1, '/edge/finalize_task1')
        self.submit = self.node.create_client(SubmitMission, '/edge/submit_mission')

    def say(self, activity, **fields):
        print(json.dumps({'run_id': self.run_id, 'activity': activity, **fields}), flush=True)

    def start(self, name, args):
        path = self.directory.parent / f'{self.run_id}.{name}.log'
        with path.open('w') as log:
            process = subprocess.Popen(['ros2', *args], stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True, cwd=ROOT)
        self.processes.append((process, path))

    def wait(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            for process, path in self.processes:
                if process.poll() is not None:
                    raise RuntimeError(f'process exited with {process.returncode}: {path}')
            state = self.samples.get('phase')
            if state and state.phase == CompetitionState.PHASE_ESTOP:
                raise RuntimeError(f'system entered ESTOP: {state.reason}')
            if predicate():
                return
        raise TimeoutError(label)

    def call(self, client, request, timeout=45):
        self.wait(client.service_is_ready, 15, f'{client.srv_name} unavailable')
        result = client.call_async(request)
        self.wait(result.done, timeout, f'{client.srv_name} timed out')
        return result.result()

    def phase(self, phase_id):
        request = SetCompetitionPhase.Request(
            run_id=self.run_id, requester_type=SetCompetitionPhase.Request.REQUESTER_OPERATOR,
            requested_phase=phase_id, requester_node=self.node.get_fully_qualified_name(),
            reason='practice rehearsal')
        until = time.monotonic() + 45
        while True:
            result = self.call(self.phase_client, request, 45 + 35 * len(self.robots))
            if result.accepted:
                return
            if result.message != 'another control operation is active' or time.monotonic() > until:
                raise RuntimeError(f'phase {phase_id}: {result.message}')
            time.sleep(0.25)

    def ready(self, r):
        msg = self.samples.get(r)
        return bool(msg and msg.localized and msg.navigation_available)

    def prepare(self):
        options = self.options
        args = ['launch', 'edgeai_system_manager', 'task1.launch.py', 'scenario:=midterm',
                'rescue:=false', 'record_bag:=false', 'headless:=true', 'rviz:=false',
                f'runs_root:={self.directory.parent}', f'run_id:={self.run_id}',
                f'world:={options.arena / "arena.sdf"}']
        waiting = self.robots[2:]
        if waiting:
            args.append('waiting_robots:=' + ','.join(waiting))
        self.start('system', args)
        self.wait(lambda: 'phase' in self.samples and self.ready('robot_1'), 150,
                  'SLAM and navigation did not become ready')
        self.phase(CompetitionState.PHASE_TASK1)
        self.say('Task 1 started')
        self.wait(lambda: self.samples['phase'].phase == CompetitionState.PHASE_HANDOVER,
                  options.task1_timeout, 'exploration did not reach handover')
        self.wait(lambda: 'map' in self.samples and json.loads(self.samples['map'].data)['frozen']
                  and 'targets' in self.samples and self.samples['targets'].task1_frozen,
                  35, 'map and target snapshot did not freeze')
        self.phase(CompetitionState.PHASE_HANDOVER)
        targets = self.samples['targets']
        response = self.call(self.finalize, FinalizeTask1.Request(
            run_id=self.run_id, map_id=targets.map_id, expected_map_revision=targets.map_revision,
            expected_target_revision=targets.revision))
        if not response.success:
            raise RuntimeError(f'finalize_task1: {response.message}')
        target_ids = {t.target_id for t in targets.targets if t.eligible_for_basic_rescue}
        self.say('Task 1 finalized', target_ids=sorted(target_ids))
        # Ground truth is used only as an acceptance reference for perception.
        truth = json.loads((options.arena / 'ground_truth.json').read_text())
        expected = {m['target_id'] for m in truth['markers'] if m['target_id']}
        self.missing_targets = sorted(expected - target_ids)
        if not target_ids or target_ids - expected:
            raise RuntimeError(f'no valid frozen targets or unexpected IDs: {sorted(target_ids)}')
        if self.missing_targets:
            self.say('Task 1 missed targets; continuing with the frozen list',
                     missing=self.missing_targets)
        if waiting:
            sys.path.insert(0, str(ROOT / 'tools'))
            from localization_check import GroundTruth, place_robots
            poses = GroundTruth()
            self.wait(lambda: all(poses.get(r) is not None for r in self.robots), 20,
                      'no Gazebo ground-truth poses for the handover')
            starts = resolve_parameters('development').require('localization.start_poses')
            place_robots(poses, {r: starts['handover_' + r] for r in waiting},
                         self.robots, self.wait)
        return target_ids

    def run(self):
        targets = self.prepare()
        self.start('allocator', [
            'launch', 'edgeai_task_coordinator', 'frozen_rescue.launch.py',
            f'run_directory:={self.directory}', 'robot_ids:=' + ','.join(self.robots)])
        fixture = str(self.directory / 'task1/targets.json')
        for executable in ('mission_interpreter', 'mission_validator'):
            package = ('edgeai_mission_interpreter' if executable == 'mission_interpreter'
                       else 'edgeai_rescue_coordinator')
            self.start(executable, ['run', package, executable, '--ros-args',
                                    '-p', f'frozen_targets:={fixture}', '-p', 'use_sim_time:=true'])
        self.phase(CompetitionState.PHASE_TASK2A)
        self.say('Task 2A started')
        self.wait(lambda: self.samples['phase'].phase == CompetitionState.PHASE_FROZEN,
                  self.options.rescue_timeout, 'Task 2A did not freeze')
        completed = self.completed(RescueEvent.STAGE_TASK2A)
        self.failed_task2a_targets = sorted(targets - completed)
        self.say('Task 2A froze', target_ids=sorted(completed),
                 failed_or_incomplete=self.failed_task2a_targets)
        self.phase(CompetitionState.PHASE_TASK2B)
        self.wait(lambda: self.samples['phase'].phase == CompetitionState.PHASE_TASK2B
                  and all(self.ready(r) for r in self.robots),
                  60, 'robots not ready for Task 2B')
        self.wait(lambda: self.rescue_status().get('status') == 'WAITING_FOR_MISSION',
                  30, 'allocator not ready for Task 2B')
        response = self.call(self.submit, SubmitMission.Request(
            run_id=self.run_id, original_instruction='Rescue all targets.'))
        if not response.accepted:
            raise RuntimeError(f'mission intake rejected: {response.message}')
        mid = response.mission_id
        self.wait(lambda: mid in self.intakes, 15, 'missing mission intake result')
        if self.intakes[mid].outcome != MissionIntakeResult.OUTCOME_ACCEPTED:
            raise RuntimeError(f'mission rejected: {to_dict(self.intakes[mid])}')
        self.say('Task 2B started', mission_id=mid)

        def terminal():
            s = self.samples.get('mission')
            return (s and s.mission_id == mid
                    and s.state in (s.STATE_COMPLETED, s.STATE_FAILED, s.STATE_ABORTED))

        self.wait(terminal, self.options.rescue_timeout, 'Task 2B did not reach a terminal state')
        if (self.samples['mission'].state != MissionExecutionState.STATE_COMPLETED
                or self.completed(RescueEvent.STAGE_TASK2B) != targets):
            raise RuntimeError(f'Task 2B did not complete: {to_dict(self.samples["mission"])}')
        self.wait(lambda: all(self.ready(r) and not self.samples[r].active_task_id
                              for r in self.robots), 20, 'robots did not release rescue actions')
        self.phase(CompetitionState.PHASE_FINISHED)
        result = {'audit': audit(self.directory, ROOT / 'schemas', require_finished=True),
                  'mission_id': mid, 'targets': sorted(targets)}
        if self.failed_task2a_targets:
            result['error'] = 'Task 2A did not complete all frozen targets; Task 2B completed'
        return result

    def completed(self, stage):
        return {e.target_id for e in self.events
                if e.scoring_stage == stage and e.event_type == RescueEvent.EVENT_COMPLETED}

    def rescue_status(self):
        msg = self.samples.get('rescue')
        return json.loads(msg.data) if msg else {}

    def report(self, result):
        data = {'run_id': self.run_id, 'robots': self.robots, 'result': result,
                'missing_task1_targets': self.missing_targets,
                'failed_or_incomplete_task2a_targets': self.failed_task2a_targets,
                'last_rescue_status': self.rescue_status(),
                'events': [to_dict(e) for e in self.events]}
        path = self.directory.parent / f'{self.run_id}.frozen-rehearsal.json'
        atomic_json(path, data)
        self.say('report', file=str(path), success='error' not in result)

    def close(self):
        # These process groups belong only to this rehearsal.
        for sig in (signal.SIGINT, signal.SIGTERM):
            for process, _ in reversed(self.processes):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    pass
            for process, _ in reversed(self.processes):
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    pass
        for process, _ in self.processes:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
        self.node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arena', type=lambda p: Path(p).resolve(), required=True)
    parser.add_argument('--robots', type=int, choices=(2, 3, 4), default=2)
    parser.add_argument('--output', type=Path, default=Path('runs/frozen-rehearsal'))
    parser.add_argument('--domain', type=int, default=77)
    parser.add_argument('--task1-timeout', type=float, default=780)
    parser.add_argument('--rescue-timeout', type=float, default=600)
    harness = Rehearsal(parser.parse_args())
    try:
        try:
            result = harness.run()
        except (Exception, KeyboardInterrupt) as error:
            result = {'error': str(error) or type(error).__name__}
            harness.say('failed', **result)
        harness.report(result)
        return int('error' in result)
    finally:
        harness.close()


if __name__ == '__main__':
    raise SystemExit(main())
