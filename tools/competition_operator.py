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

"""Send one explicit operator command to an already launched competition run."""

import argparse
import json
import os
import re
import sys
import time
from pathlib import Path


def matching_freeze(run_id, state, mapping, targets):
    """Match the two frozen artifacts to the manager's current revision."""
    return (
        state.phase == 2 and not state.estop_active
        and state.run_id == targets.run_id == mapping['run_id'] == run_id
        and state.map_id == targets.map_id == mapping['map_id']
        and state.map_revision == targets.map_revision == mapping['revision']
        and state.target_revision == targets.revision
        and mapping['frozen'] and targets.task1_frozen)


def main():
    """Read or control one explicitly identified contest run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('status', 'start', 'handover', 'finalize',
                                            'task2a', 'freeze', 'task2b', 'finish', 'estop'))
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--runs-root', type=Path, default=Path('runs'))
    parser.add_argument('--reason', default='operator command')
    options = parser.parse_args()

    from edgeai_common.config import load_runtime_contract
    from edgeai_common.qos import qos_from_contract
    from edgeai_robot_interfaces.msg import CompetitionState, TargetArray
    from edgeai_robot_interfaces.srv import FinalizeTask1, SetCompetitionPhase
    from edgeai_system_manager.audit import audit
    import rclpy
    from std_msgs.msg import String

    if not re.fullmatch(r'run-\d{8}T\d{6}Z-[a-f0-9]{8}', options.run_id):
        parser.error('invalid --run-id')
    phases = {'start': 1, 'handover': 2, 'task2a': 3, 'freeze': 4,
              'task2b': 5, 'finish': 6, 'estop': 7}
    received = {}
    contract = load_runtime_contract()
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')
    rclpy.init()
    try:
        node = rclpy.create_node('competition_operator')
    except RuntimeError as error:
        print(f'could not start ROS operator: {error}', file=sys.stderr)
        rclpy.try_shutdown()
        return 1

    def receive(key, message):
        value = json.loads(message.data) if key == 'map' else message
        run_id = value['run_id'] if key == 'map' else value.run_id
        if run_id == options.run_id:
            received[key] = value

    def wait(predicate, seconds, description):
        deadline = time.monotonic() + seconds
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(description)
            rclpy.spin_once(node, timeout_sec=min(remaining, 0.1))

    def call(service_type, topic, request):
        client = node.create_client(service_type, topic)
        if not client.wait_for_service(timeout_sec=5):
            raise TimeoutError('service unavailable: ' + topic)
        future = client.call_async(request)
        wait(future.done, 190, 'service response timed out: ' + topic)
        return future.result()

    try:
        # Send ESTOP without waiting for status. The server still checks the run ID.
        if options.command in ('status', 'finalize'):
            node.create_subscription(
                CompetitionState, '/competition/state', lambda msg: receive('state', msg),
                qos_from_contract(contract, 'control_state'))
            wait(lambda: 'state' in received, 5, 'no competition state for requested run')
        if options.command == 'status':
            state = received['state']
            result = {'run_id': state.run_id, 'phase': state.phase,
                      'phase_sequence': state.phase_sequence, 'map_id': state.map_id,
                      'map_revision': state.map_revision, 'target_revision': state.target_revision,
                      'estop': state.estop_active, 'reason': state.reason}
        elif options.command == 'finalize':
            snapshot = qos_from_contract(contract, 'snapshot')
            node.create_subscription(
                TargetArray, '/edge/targets', lambda msg: receive('targets', msg), snapshot)
            node.create_subscription(
                String, '/edge/map_metadata', lambda msg: receive('map', msg), snapshot)
            wait(lambda: 'map' in received and 'targets' in received and matching_freeze(
                options.run_id, received['state'], received['map'], received['targets']),
                45, 'matching frozen snapshots unavailable')
            targets = received['targets']
            request = FinalizeTask1.Request(
                run_id=options.run_id, map_id=targets.map_id,
                expected_map_revision=targets.map_revision,
                expected_target_revision=targets.revision)
            response = call(FinalizeTask1, '/edge/finalize_task1', request)
            if not response.success:
                raise RuntimeError(response.message)
            result = {'manifest': response.manifest_uri, 'map': response.map_yaml_uri,
                      'targets': response.target_list_uri}
        else:
            request = SetCompetitionPhase.Request(
                run_id=options.run_id, requested_phase=phases[options.command],
                requester_type=SetCompetitionPhase.Request.REQUESTER_OPERATOR,
                requester_node='/competition_operator', reason=options.reason)
            response = call(SetCompetitionPhase, '/competition/set_phase', request)
            if not response.accepted:
                raise RuntimeError(response.message)
            result = {'phase': response.current_phase, 'phase_sequence': response.phase_sequence,
                      'message': response.message}
            if options.command == 'finish':
                result['audit'] = audit(options.runs_root / options.run_id,
                                        Path.cwd() / 'schemas', require_finished=True)
        print(json.dumps(result, ensure_ascii=False))
        return 0
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
        print(f'{options.command}: {error}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
