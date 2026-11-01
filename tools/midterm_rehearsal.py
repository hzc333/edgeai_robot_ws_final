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

"""
Full midterm rehearsal with every module: P1 robots, P2 phases/SLAM/exploration, P3
detection, P4 rescue (`task1.launch.py scenario:=midterm`).

This driver only plays the operator, as P2's validate_sim does:

1. TASK1. P2 explores autonomously, visits the viewpoints of markers P3 has not
   detected yet, drives robot_1 back to the start area and requests HANDOVER.
2. HANDOVER. Waits for the frozen map and P3's frozen target snapshot, then calls
   /edge/finalize_task1.
3. TASK2A. P2 relocalizes robot_1 (last SLAM pose) and robot_2 (its start pose);
   P4 rescues S1 and C1 and requests FROZEN.
4. FINISHED, then P2's audit of the sealed run.

Detected targets are compared with the marker centres of P4's midterm config.
Run from the repository root after `pixi run build`:

    pixi run bash -c 'source install/setup.bash && python tools/midterm_rehearsal.py --gui'

Reports go to <runs-root>/<run_id>.rehearsal.json; exit status 0 on success.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs-root', default='runs/midterm-rehearsal')
    parser.add_argument('--domain', type=int, default=77, help='isolated ROS_DOMAIN_ID')
    parser.add_argument('--gui', action='store_true', help='open Gazebo and RViz')
    parser.add_argument('--task1-timeout', type=float, default=780.0,
                        help='TASK1 limit [s wall]')
    parser.add_argument('--rescue-timeout', type=float, default=420.0,
                        help='TASK2A limit [s wall]')
    parser.add_argument('--debug-on-fail', action='store_true',
                        help='dump robot_2 AMCL pose and TF before teardown on failure')
    options = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(options.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')

    from ament_index_python.packages import get_package_share_directory
    from edgeai_common.config import load_runtime_contract
    from edgeai_common.qos import qos_from_contract
    from edgeai_robot_interfaces.msg import CompetitionState, RobotState, TargetArray
    from edgeai_robot_interfaces.srv import FinalizeTask1, SetCompetitionPhase
    from edgeai_system_manager.audit import audit
    from edgeai_system_manager.storage import atomic_json, new_run_id
    import rclpy
    from std_msgs.msg import String
    import yaml

    root = Path(options.runs_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    run_id = new_run_id()
    log = (root / (run_id + '.launch.log')).open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'edgeai_system_manager', 'task1.launch.py', 'scenario:=midterm',
        'runs_root:=' + str(root), 'run_id:=' + run_id,
        'headless:=' + str(not options.gui).lower(), 'rviz:=' + str(options.gui).lower()],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('midterm_rehearsal_operator')
    received = {}
    contract = load_runtime_contract()
    for kind, topic, key, qos in [
            (CompetitionState, '/competition/state', 'state', 'control_state'),
            (RobotState, '/robot_1/state', 'robot_1', 'robot_state'),
            (RobotState, '/robot_2/state', 'robot_2', 'robot_state'),
            (TargetArray, '/edge/targets', 'targets', 'snapshot'),
            (String, '/edge/map_metadata', 'map', 'snapshot'),
            (String, '/edge/midterm_rescue/status', 'p4', 'snapshot')]:
        node.create_subscription(kind, topic, lambda msg, key=key: received.update({key: msg}),
                                 qos_from_contract(contract, qos))
    phase_client = node.create_client(SetCompetitionPhase, '/competition/set_phase')
    finalize_client = node.create_client(FinalizeTask1, '/edge/finalize_task1')
    markers = {t['id']: t['center'] for t in yaml.safe_load(
        (Path(get_package_share_directory('edgeai_task_coordinator')) / 'config'
         / 'midterm.yaml').read_text())['targets']}
    report = {'run_id': run_id, 'success': False, 'times_s': {}}
    p4_seen = []

    def say(text, **details):
        print(json.dumps({'step': text, **details}, ensure_ascii=False), flush=True)

    def wait(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if 'p4' in received:
                data = json.loads(received['p4'].data)
                key = (data.get('status'), data.get('target_id'))
                if not p4_seen or p4_seen[-1] != key:
                    p4_seen.append(key)
                    say('P4', status=key[0], target=key[1])
            if process.poll() is not None:
                raise RuntimeError('launch exited; inspect the launch log')
            if 'state' in received and received['state'].phase == 7:
                raise RuntimeError('ESTOP: ' + received['state'].reason)
            if predicate():
                return
        raise TimeoutError(label)

    def call(client, request, timeout=45):
        wait(client.service_is_ready, 10, 'service unavailable')
        future = client.call_async(request)
        wait(future.done, timeout, 'service timed out')
        return future.result()

    def phase(destination):
        request = SetCompetitionPhase.Request()
        request.run_id, request.requested_phase, request.requester_type = run_id, destination, 1
        request.requester_node, request.reason = '/midterm_rehearsal_operator', 'rehearsal'
        deadline = time.monotonic() + 45
        response = call(phase_client, request)
        while (not response.accepted and response.message == 'another control operation is active'
               and time.monotonic() < deadline):
            time.sleep(0.25)
            response = call(phase_client, request)
        if not response.accepted:
            raise RuntimeError(f'phase {destination}: {response.message}')

    def ready(robot_id):
        state = received.get(robot_id)
        return bool(state and state.localized and state.navigation_available)

    started = time.monotonic()
    try:
        wait(lambda: 'state' in received and ready('robot_1'), 150,
             'SLAM/Nav2 did not become ready')
        say('ready; starting TASK1 (autonomous exploration)')
        phase(1)
        task1 = time.monotonic()
        wait(lambda: received['state'].phase == 2, options.task1_timeout,
             'exploration did not end')
        report['task1_end_reason'] = received['state'].reason
        report['times_s']['task1'] = round(time.monotonic() - task1, 1)
        say('TASK1 ended', reason=report['task1_end_reason'],
            seconds=report['times_s']['task1'])
        handover = time.monotonic()
        wait(lambda: 'map' in received and json.loads(received['map'].data)['frozen']
             and 'targets' in received and received['targets'].task1_frozen,
             30, 'map/target freeze not confirmed')
        phase(2)
        targets = received['targets']
        found = {}
        for target in targets.targets:
            p = target.marker_pose.pose.position
            centre = markers.get(target.target_id)
            found[target.target_id] = {
                'xy': [round(p.x, 3), round(p.y, 3)],
                'error_m': round(math.hypot(p.x - centre[0], p.y - centre[1]), 3)
                if centre else None}
        report['task1_targets'] = found
        say('Task 1 targets', targets=found)
        request = FinalizeTask1.Request()
        request.run_id, request.map_id = run_id, targets.map_id
        request.expected_map_revision = targets.map_revision
        request.expected_target_revision = targets.revision
        response = call(finalize_client, request)
        if not response.success:
            raise RuntimeError('finalize_task1: ' + response.message)
        phase(3)
        # P4 may already have requested FROZEN (phase 4) when it fails closed.
        wait(lambda: received['state'].phase in (3, 4) and ready('robot_1')
             and ready('robot_2'), 60, 'robots not relocalized for TASK2A')
        report['times_s']['handover'] = round(time.monotonic() - handover, 1)
        say('TASK2A: P4 rescue', handover_seconds=report['times_s']['handover'])
        rescue = time.monotonic()
        wait(lambda: received['state'].phase == 4, options.rescue_timeout,
             'P4 did not finish the rescue')
        report['rescue_reason'] = received['state'].reason
        report['times_s']['task2a'] = round(time.monotonic() - rescue, 1)
        report['times_s']['total'] = round(time.monotonic() - task1, 1)
        rescued = bool(p4_seen and p4_seen[-1][0] == 'DONE')
        say('FROZEN', reason=report['rescue_reason'], p4_done=rescued,
            seconds=report['times_s']['task2a'], total_seconds=report['times_s']['total'])
        phase(6)
        report['audit'] = audit(root / run_id, Path.cwd() / 'schemas', require_finished=True)
        report['success'] = rescued and set(found) >= set(markers)
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error)
        report['robots'] = {
            r: {'localized': m.localized, 'navigation_available': m.navigation_available,
                'mode': m.mode, 'xy': [round(m.pose.pose.position.x, 2),
                                       round(m.pose.pose.position.y, 2)]}
            for r, m in ((r, received.get(r)) for r in ('robot_1', 'robot_2')) if m}
        if 'p4' in received:
            report['p4_last_status'] = json.loads(received['p4'].data)
        say('failed', error=str(error), robots=report['robots'],
            p4_error=report.get('p4_last_status', {}).get('error'))
        if options.debug_on_fail and process.poll() is None:
            # Evidence from the still running system before it is torn down.
            for command in (
                    ['ros2', 'topic', 'echo', '--once', '--qos-durability', 'transient_local',
                     '/robot_2/amcl_pose'],
                    ['ros2', 'run', 'tf2_ros', 'tf2_echo', 'map', 'robot_2/base_footprint'],
                    ['ros2', 'topic', 'info', '-v', '/robot_2/amcl_pose'],
                    [sys.executable, str(Path(__file__).with_name('probe_localized.py'))]):
                try:
                    out = subprocess.run(command, capture_output=True, text=True, timeout=12)
                    text = out.stdout
                except subprocess.TimeoutExpired as expired:
                    text = (expired.stdout or b'').decode(errors='replace')
                (root / (run_id + '.debug.txt')).open('a').write(
                    '$ ' + ' '.join(command) + '\n' + text[-3000:] + '\n')
    finally:
        report['p4_status_sequence'] = p4_seen
        report['wall_s'] = round(time.monotonic() - started, 1)
        atomic_json(root / (run_id + '.rehearsal.json'), report)
        if not report['success']:
            try:
                phase(7)
            except Exception:
                pass
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                pass
        # gz's Ruby launcher may exit while its server remains in our process group.
        try:
            os.killpg(process.pid, signal.SIGTERM)
            time.sleep(0.5)
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'passed={report["success"]}; report in {root / (run_id + ".rehearsal.json")}')
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
