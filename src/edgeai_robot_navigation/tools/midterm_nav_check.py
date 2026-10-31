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
P1 navigation regression in the midterm arena (two robots, real Nav2/AMCL).

Starts `midterm_sim.launch.py part:=2` headless in its own DDS domain and Gazebo
partition, plays the competition state (TASK2A) itself, relocalizes both robots
at their midterm start poses and then runs `--rounds` rounds of one scenario:

cross   both robots swap the diagonals of the open area west of O2, so their paths
        cross (like leaving the start area for S1). Passes when every goal
        succeeds and the footprints always stay more than 0.05 m apart.
headon  they swap places head-on through the 1.45 m corridor south of O2. Local
        avoidance cannot resolve this (P4 schedules around it); passes when the
        footprints never touch, BLOCKED results are expected.

For every round it reports the action results, the wall-clock time, the final
error and pose from Gazebo ground truth and the smallest gap between the two real
footprints (0 means they touched). Run from the repository root after
`pixi run build`, inside `pixi run bash -c 'source install/setup.bash && ...'`:

    python3 src/edgeai_robot_navigation/tools/midterm_nav_check.py --output runs/p1-nav-check
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROBOTS = ('robot_1', 'robot_2')
SCENARIOS = {
    # (robot_1 start, robot_2 start): odd rounds swap to the other pair, even rounds back.
    'cross': (((0.6, 0.8), (1.5, 0.8)), ((1.5, 2.4), (0.6, 2.4))),
    'headon': (((0.6, 1.0), (3.0, 1.0)), ((3.0, 1.0), (0.6, 1.0))),
}
MIN_GAP_M = {'cross': 0.05, 'headon': 0.0}
FOOTPRINT = ((0.115, 0.175), (0.115, -0.175), (-0.315, -0.175), (-0.315, 0.175))
TERMINAL = {1: 'SUCCEEDED', 2: 'FAILED', 3: 'CANCELLED', 4: 'BLOCKED'}


def footprint_gap(pose_a, pose_b):
    """Smallest distance between two robot footprints at (x, y, yaw); 0 if they overlap."""
    def polygon(pose):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * u - s * v, y + s * u + c * v) for u, v in FOOTPRINT]

    def inside(point, poly):
        signs = [(poly[(i + 1) % 4][0] - poly[i][0]) * (point[1] - poly[i][1])
                 - (poly[(i + 1) % 4][1] - poly[i][1]) * (point[0] - poly[i][0])
                 for i in range(4)]
        return all(v >= 0 for v in signs) or all(v <= 0 for v in signs)

    def segment_distance(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
        return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)

    a, b = polygon(pose_a), polygon(pose_b)
    if any(inside(v, b) for v in a) or any(inside(v, a) for v in b):
        return 0.0
    return min(segment_distance(v, poly[i], poly[(i + 1) % 4])
               for v, poly in [(v, b) for v in a] + [(v, a) for v in b] for i in range(4))


def gazebo_pose(robot_id):
    for _ in range(5):          # gz model occasionally answers empty
        out = subprocess.run(['gz', 'model', '-m', robot_id, '-p'],
                             capture_output=True, text=True).stdout
        if 'XYZ' in out:
            lines = out.split('XYZ')[1].splitlines()
            x, y, _ = (float(v) for v in lines[1].strip(' []').split())
            return x, y, float(lines[2].strip(' []').split()[2])
        time.sleep(0.3)
    raise RuntimeError(f'no Gazebo pose for {robot_id}')


def run(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter

    from builtin_interfaces.msg import Duration
    from edgeai_common.config import load_runtime_contract
    from edgeai_common.qos import qos_from_contract
    from edgeai_robot_interfaces.action import ExecuteNavigationGoal
    from edgeai_robot_interfaces.msg import CompetitionState, RobotState
    from edgeai_robot_interfaces.srv import Relocalize

    config = Path(os.environ.get('EDGEAI_CONFIG_DIR', 'config')).resolve()
    rclpy.init()
    node = Node('p1_midterm_nav_check',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    state_pub = node.create_publisher(
        CompetitionState, '/competition/state',
        qos_from_contract(load_runtime_contract(config), 'control_state'))

    def publish_state():
        message = CompetitionState(run_id='run-p1-nav-check', map_id='midterm',
                                   map_revision=1, phase=CompetitionState.PHASE_TASK2A,
                                   phase_sequence=1)
        message.header.stamp = node.get_clock().now().to_msg()
        state_pub.publish(message)
    node.create_timer(0.2, publish_state)
    states = {}
    for robot_id in ROBOTS:
        node.create_subscription(RobotState, f'/{robot_id}/state',
                                 lambda m, r=robot_id: states.__setitem__(r, m), 10)
    # Create every client before the executor thread spins (rclpy is not thread-safe here).
    relocalize = {r: node.create_client(Relocalize, f'/{r}/relocalize') for r in ROBOTS}
    clients = {r: ActionClient(node, ExecuteNavigationGoal, f'/{r}/execute_navigation_goal')
               for r in ROBOTS}
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def wait(predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.05)
        return False

    report = {'rounds': [], 'relocalize': {}}
    for robot_id in ROBOTS:
        client = relocalize[robot_id]
        if not client.wait_for_service(timeout_sec=60):
            raise RuntimeError(f'{robot_id} relocalize service missing')
        future = client.call_async(Relocalize.Request(
            run_id='run-p1-nav-check', map_id='midterm', map_revision=1,
            mode=Relocalize.Request.MODE_KNOWN_START_POSE,
            start_pose_id=f'midterm_{robot_id}', timeout=Duration(sec=30)))
        wait(future.done, 60)
        result = future.result()
        report['relocalize'][robot_id] = {'success': bool(result and result.success),
                                          'message': result.message if result else 'no reply'}
        print(f'{robot_id} relocalize: {report["relocalize"][robot_id]}', flush=True)
    wait(lambda: all(r in states and states[r].localized and states[r].navigation_available
                     for r in ROBOTS), 30)
    for client in clients.values():
        client.wait_for_server(timeout_sec=30)

    def send(robot_id, goal_xy, tag):
        goal = ExecuteNavigationGoal.Goal(run_id='run-p1-nav-check', request_id=tag,
                                          map_id='midterm', map_revision=1,
                                          xy_tolerance_m=0.10)
        goal.timeout.sec = 150
        goal.goal_pose.header.frame_id = 'map'
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = goal_xy
        goal.goal_pose.pose.orientation.w = 1.0
        future = clients[robot_id].send_goal_async(goal)
        wait(future.done, 10)
        return future.result()

    def result(handle, timeout=180):
        if handle is None or not handle.accepted:
            return None
        future = handle.get_result_async()
        wait(future.done, timeout)
        return future.result().result if future.done() else None

    home, away = SCENARIOS[args.scenario]
    # Setup: drive to the start pair one after the other (robot_2 first, it is ahead).
    for robot_id, goal_xy in (('robot_2', home[1]), ('robot_1', home[0])):
        outcome = result(send(robot_id, goal_xy, f'setup-{robot_id}'))
        print(f'setup {robot_id}: {TERMINAL.get(outcome.terminal_state) if outcome else None}',
              flush=True)

    gap = {'value': math.inf, 'run': False}

    def monitor():
        while True:
            if gap['run']:
                try:
                    gap['value'] = min(gap['value'], footprint_gap(gazebo_pose('robot_1'),
                                                                   gazebo_pose('robot_2')))
                except RuntimeError:
                    pass
            time.sleep(0.2)
    threading.Thread(target=monitor, daemon=True).start()

    ok = True
    for index in range(1, args.rounds + 1):
        pair = away if index % 2 else home
        goals = {'robot_1': pair[0], 'robot_2': pair[1]}
        gap.update(value=math.inf, run=True)
        start = time.monotonic()
        handles = {r: send(r, goals[r], f'round{index}-{r}') for r in ROBOTS}
        outcomes = {r: result(handles[r]) for r in ROBOTS}
        gap['run'] = False
        entry = {'round': index, 'wall_s': round(time.monotonic() - start, 1),
                 'min_footprint_gap_m': round(gap['value'], 3), 'robots': {}}
        for robot_id in ROBOTS:
            outcome = outcomes[robot_id]
            x, y, yaw = gazebo_pose(robot_id)
            terminal = TERMINAL.get(outcome.terminal_state) if outcome else 'NO_RESULT'
            entry['robots'][robot_id] = {
                'terminal': terminal,
                'detail': outcome.reason.detail if outcome else '',
                'true_error_m': round(math.hypot(x - goals[robot_id][0],
                                                 y - goals[robot_id][1]), 3),
                'true_pose': [round(x, 3), round(y, 3), round(yaw, 3)]}
            ok = ok and (terminal == 'SUCCEEDED' or args.scenario == 'headon')
        ok = ok and gap['value'] > MIN_GAP_M[args.scenario]
        report['rounds'].append(entry)
        print(json.dumps(entry), flush=True)
    report['scenario'] = args.scenario
    report['passed'] = ok
    executor.shutdown()
    rclpy.try_shutdown()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', required=True, type=Path, help='new report directory')
    parser.add_argument('--scenario', choices=sorted(SCENARIOS), default='cross')
    parser.add_argument('--rounds', type=int, default=4, help='place swaps')
    parser.add_argument('--domain', type=int, default=75, help='isolated ROS_DOMAIN_ID')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      GZ_PARTITION=f'p1-nav-check-{os.getpid()}')
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')
    log = open(args.output / 'launch.log', 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'edgeai_robot_bringup', 'midterm_sim.launch.py',
                               'part:=2', 'headless:=true'],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(25)
        report = run(args)
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
        log.close()
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'passed={report["passed"]}; report in {args.output / "report.json"}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
