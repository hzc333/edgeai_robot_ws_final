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

"""Run an isolated, bounded Gazebo/P2/AMCL acceptance rehearsal.

Uses an explicitly empty development target fixture. No perception claims.
The driver only starts Task 1; every exploration waypoint is chosen by P2.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.action import ExecuteNavigationGoal
from edgeai_robot_interfaces.msg import CompetitionState, ExplorationStatus, RobotState, TargetArray
from edgeai_robot_interfaces.srv import FinalizeTask1, SetCompetitionPhase
import rclpy
from rclpy.action import ActionClient
from std_msgs.msg import String

from .audit import audit
from .storage import atomic_json, new_run_id


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs-root', default='runs/p2-validation')
    parser.add_argument('--domain', type=int, default=74)
    parser.add_argument('--timeout', type=float, default=750)
    options = parser.parse_args(args)
    os.environ['ROS_DOMAIN_ID'] = str(options.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    root = Path(options.runs_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    run_id = new_run_id()
    os.environ['GZ_PARTITION'] = 'p2-' + run_id
    log = (root / (run_id + '.launch.log')).open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'edgeai_system_manager', 'task1.launch.py',
        'runs_root:=' + str(root), 'run_id:=' + run_id, 'headless:=true',
        'mock_targets:=true', 'record_bag:=true'], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('p2_simulation_verifier')
    received = {}
    samples = []
    contract = load_runtime_contract()
    for kind, topic, key, qos in [
            (CompetitionState, '/competition/state', 'state', 'control_state'),
            (RobotState, '/robot_1/state', 'robot', 'robot_state'),
            (ExplorationStatus, '/edge/exploration_status', 'exploration', 'snapshot'),
            (TargetArray, '/edge/targets', 'targets', 'snapshot'),
            (String, '/edge/map_metadata', 'map', 'snapshot')]:
        node.create_subscription(kind, topic, lambda msg, key=key: received.update({key: msg}),
                                 qos_from_contract(contract, qos))
    phase_client = node.create_client(SetCompetitionPhase, '/competition/set_phase')
    finalize_client = node.create_client(FinalizeTask1, '/edge/finalize_task1')
    report = {'run_id': run_id, 'gazebo_partition': os.environ['GZ_PARTITION'],
              'perception': 'empty development fixture', 'success': False}

    def wait(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        last_sample = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_sample > 1 and 'robot' in received:
                robot = received['robot']
                samples.append({'wall_monotonic': now, 'x': robot.pose.pose.position.x,
                                'y': robot.pose.pose.position.y, 'localized': robot.localized,
                                'phase': received['state'].phase if 'state' in received else None})
                last_sample = now
            if process.poll() is not None:
                raise RuntimeError('launch exited; inspect launch log')
            if predicate():
                return
        raise TimeoutError(label)

    def call(client, request, timeout=35):
        wait(client.service_is_ready, 10, 'service unavailable')
        future = client.call_async(request)
        wait(future.done, timeout, 'service timed out')
        return future.result()

    def phase(destination):
        request = SetCompetitionPhase.Request()
        request.run_id, request.requested_phase, request.requester_type = run_id, destination, 1
        request.requester_node, request.reason = '/p2_simulation_verifier', 'acceptance rehearsal'
        deadline = time.monotonic() + 45
        response = call(phase_client, request, 45)
        while not response.accepted and response.message == 'another control operation is active':
            if time.monotonic() >= deadline:
                break
            until = time.monotonic() + 0.25
            wait(lambda: time.monotonic() >= until, 1, 'retry delay')
            response = call(phase_client, request, 45)
        if not response.accepted:
            raise RuntimeError(response.message)
        return response

    try:
        wait(lambda: 'state' in received and 'robot' in received
             and received['robot'].localized and received['robot'].navigation_available,
             90, 'SLAM/Nav2 did not become ready')
        print('SLAM/Nav2 ready; starting autonomous Task 1', flush=True)
        phase(1)
        wait(lambda: received['state'].phase in (2, 7), options.timeout,
             'autonomous exploration did not end')
        if received['state'].phase == 7:
            raise RuntimeError('ESTOP: ' + received['state'].reason)
        report['exploration_end_reason'] = received['state'].reason
        print('Autonomous exploration ended; waiting for frozen map', flush=True)
        wait(lambda: 'map' in received and json.loads(received['map'].data)['frozen']
             and 'targets' in received and received['targets'].task1_frozen,
             30, 'map/target freeze not confirmed')
        # Same-phase request waits for final safe acknowledgement without re-entering motion.
        phase(2)
        targets = received['targets']
        request = FinalizeTask1.Request()
        request.run_id, request.map_id = run_id, targets.map_id
        request.expected_map_revision, request.expected_target_revision = (
            targets.map_revision, targets.revision)
        response = call(finalize_client, request)
        if not response.success:
            raise RuntimeError(response.message)
        print('Task 1 committed; switching to AMCL', flush=True)
        phase(3)
        wait(lambda: received['state'].phase == 3 and received['robot'].localized,
             10, 'AMCL handover not ready')
        navigation = ActionClient(node, ExecuteNavigationGoal, '/robot_1/execute_navigation_goal')
        wait(navigation.server_is_ready, 5, 'navigation action missing')
        goal = ExecuteNavigationGoal.Goal()
        goal.run_id, goal.map_id = run_id, targets.map_id
        goal.map_revision = targets.map_revision
        goal.request_id, goal.purpose = 'p2-reload-navigation', 3
        goal.goal_pose.header.frame_id = 'map'
        goal.goal_pose.pose.orientation.w = 1.0
        goal.xy_tolerance_m = 0.15
        goal.timeout.sec = 120
        future = navigation.send_goal_async(goal)
        wait(future.done, 5, 'navigation acceptance timeout')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('frozen-map navigation rejected')
        future = handle.get_result_async()
        wait(future.done, 130, 'frozen-map navigation timeout')
        result = future.result().result
        if not result.success:
            raise RuntimeError('frozen-map navigation failed: ' + result.reason.detail)
        report['frozen_map_navigation'] = True
        print('Frozen-map navigation passed; sealing evidence', flush=True)
        phase(4)
        phase(6)
        report['audit'] = audit(root / run_id, Path.cwd() / 'schemas', require_finished=True)
        report['success'] = True
        print('P2 rehearsal passed', flush=True)
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error)
        print('P2 rehearsal failed: ' + str(error), flush=True)
    finally:
        report['samples'] = samples
        if 'state' in received:
            report['last_phase'] = received['state'].phase
        atomic_json(root / (run_id + '.validation.json'), report)
        if not report['success']:
            try:
                phase(7)
            except Exception:
                pass
        if process.poll() is None:
            os.kill(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        # gz's Ruby launcher may exit while its server remains in our process group.
        # This session belongs exclusively to this rehearsal (start_new_session=True).
        try:
            os.killpg(process.pid, signal.SIGTERM)
            time.sleep(0.5)
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
    if not report['success']:
        raise SystemExit(1)
