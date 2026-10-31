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
P1 check of the midterm Part 1 -> Part 2 switch (real SLAM, Nav2, AMCL, Gazebo).

The orchestration belongs to P2; this tool plays it only to check the P1 side:

1. Part 1: `midterm_sim.launch.py part:=1` (robot_1 SLAM, robot_2 parked) plus
   SLAM Toolbox. robot_1 drives across the arena and back to its start pose.
   SLAM anchors its map at robot_1's start, so goals are given in that frame.
2. Switch (timed): HANDOVER, stop SLAM, load the pre-built midterm map (arena
   frame), relocalize robot_1 and robot_2 at their midterm start poses, TASK2A,
   wait until both report localized and navigation available.
3. Part 2 check: both robots drive to arena goals; errors against ground truth.
   With --rescue, P4's coordinator runs the full Part 2 rescue (S1, then C1 with
   both robots) instead; this tool grants its FROZEN request as P2 would and
   reports the Part 2 and total wall times.

Run from the repository root after `pixi run build`, inside
`pixi run bash -c 'source install/setup.bash && ...'`:

    python3 src/edgeai_robot_navigation/tools/midterm_switch_check.py --output runs/p1-switch
    python3 src/edgeai_robot_navigation/tools/midterm_switch_check.py --rescue --gui \
        --output runs/p1-full

Exit status 0 when every step succeeded and the switch took at most --budget seconds.
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
START = {'robot_1': (0.40, 0.5, math.pi / 2),             # config/sim/robots_midterm.yaml
         'robot_2': (1.05, 0.5, math.pi / 2)}
EXPLORE = (2.9, 2.6)                    # arena goal for robot_1 in Part 1 (between O2 and W2)
PART2_GOALS = {'robot_1': (0.6, 2.2), 'robot_2': (1.5, 1.2)}
TERMINAL = {1: 'SUCCEEDED', 2: 'FAILED', 3: 'CANCELLED', 4: 'BLOCKED'}


def gazebo_xy(robot_id):
    for _ in range(5):
        out = subprocess.run(['gz', 'model', '-m', robot_id, '-p'],
                             capture_output=True, text=True).stdout
        if 'XYZ' in out:
            x, y, _ = (float(v) for v in out.split('XYZ')[1].splitlines()[1].strip(' []').split())
            return x, y
        time.sleep(0.3)
    raise RuntimeError(f'no Gazebo pose for {robot_id}')


def popen(command, log_path):
    return subprocess.Popen(command, stdout=open(log_path, 'w'), stderr=subprocess.STDOUT,
                            start_new_session=True)


def stop(process, timeout=15):
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)


def run(args, processes):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    from nav_msgs.msg import OccupancyGrid
    from ament_index_python.packages import get_package_share_directory
    from builtin_interfaces.msg import Duration
    from edgeai_common.config import load_runtime_contract
    from edgeai_common.qos import qos_from_contract
    from edgeai_robot_interfaces.action import ExecuteNavigationGoal
    from edgeai_robot_interfaces.msg import CompetitionState, RobotPhaseAck, RobotState
    from edgeai_robot_interfaces.srv import Relocalize, SetCompetitionPhase
    from std_msgs.msg import String

    config = Path(os.environ.get('EDGEAI_CONFIG_DIR', 'config')).resolve()
    rclpy.init()
    node = Node('p1_midterm_switch_check',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    phase = {'value': CompetitionState.PHASE_TASK1, 'sequence': 1}
    state_pub = node.create_publisher(
        CompetitionState, '/competition/state',
        qos_from_contract(load_runtime_contract(config), 'control_state'))

    def publish_state():
        message = CompetitionState(run_id='run-p1-switch', map_id='midterm', map_revision=1,
                                   phase=phase['value'], phase_sequence=phase['sequence'])
        message.header.stamp = node.get_clock().now().to_msg()
        state_pub.publish(message)
    node.create_timer(0.2, publish_state)
    states = {}
    for robot_id in ROBOTS:
        node.create_subscription(RobotState, f'/{robot_id}/state',
                                 lambda m, r=robot_id: states.__setitem__(r, m), 10)
    acks = {}
    ack_qos = qos_from_contract(load_runtime_contract(config), 'robot_state')
    for robot_id in ROBOTS:
        node.create_subscription(
            RobotPhaseAck, f'/{robot_id}/phase_ack',
            lambda m, r=robot_id: acks.__setitem__(r, m.phase_sequence), ack_qos)
    maps = []
    node.create_subscription(OccupancyGrid, '/map', maps.append, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    relocalize = {r: node.create_client(Relocalize, f'/{r}/relocalize') for r in ROBOTS}
    rescue = {'freeze': None, 'status': {}}

    def set_phase(request, response):
        # Stand-in for P2: grant P4's FROZEN request once both rescues are done.
        response.accepted = (request.run_id == 'run-p1-switch'
                             and request.requested_phase == CompetitionState.PHASE_FROZEN)
        if response.accepted and phase['value'] != CompetitionState.PHASE_FROZEN:
            rescue['freeze'] = {'wall': time.monotonic(), 'reason': request.reason}
            phase.update(value=CompetitionState.PHASE_FROZEN, sequence=phase['sequence'] + 1)
        response.current_phase, response.phase_sequence = phase['value'], phase['sequence']
        return response
    node.create_service(SetCompetitionPhase, '/competition/set_phase', set_phase)

    def on_status(msg):
        data = json.loads(msg.data)
        key = (data.get('status'), data.get('target_id'))
        if key != rescue['status'].get('key'):
            print(json.dumps({'p4': data.get('status'), 'target': data.get('target_id'),
                              'error': data.get('error', '')}), flush=True)
        rescue['status'] = dict(data, key=key)
    node.create_subscription(String, '/edge/midterm_rescue/status', on_status,
                             qos_from_contract(load_runtime_contract(config), 'snapshot'))
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

    def ready(robot_id):
        state = states.get(robot_id)
        return bool(state and state.localized and state.navigation_available)

    def navigate(robot_id, xy, tag, purpose):
        goal = ExecuteNavigationGoal.Goal(run_id='run-p1-switch', request_id=tag,
                                          map_id='midterm', map_revision=1,
                                          purpose=purpose, xy_tolerance_m=0.10)
        goal.timeout.sec = 150
        goal.goal_pose.header.frame_id = 'map'
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = xy
        goal.goal_pose.pose.orientation.w = 1.0
        future = clients[robot_id].send_goal_async(goal)
        wait(future.done, 10)
        handle = future.result()
        if handle is None or not handle.accepted:
            return 'REJECTED'
        result = handle.get_result_async()
        wait(result.done, 180)
        return TERMINAL.get(result.result().result.terminal_state) if result.done() else 'TIMEOUT'

    report = {'steps': []}

    def step(name, ok, **details):
        report['steps'].append({'step': name, 'ok': bool(ok), **details})
        print(json.dumps(report['steps'][-1]), flush=True)
        return ok

    ok = step('part1 robot_1 ready under SLAM', wait(lambda: ready('robot_1'), 90))
    part1_started = time.monotonic()
    # SLAM anchors its map at robot_1's start pose: convert arena goals into it.
    ox, oy, oyaw = START['robot_1']
    c, s = math.cos(oyaw), math.sin(oyaw)
    for tag, (x, y) in (('explore', EXPLORE), ('return to start', START['robot_1'][:2])):
        dx, dy = x - ox, y - oy
        outcome = navigate('robot_1', (c * dx + s * dy, -s * dx + c * dy), tag,
                           ExecuteNavigationGoal.Goal.PURPOSE_EXPLORATION)
        tx, ty = gazebo_xy('robot_1')
        ok &= step(f'part1 {tag}', outcome == 'SUCCEEDED', result=outcome,
                   true_error_m=round(math.hypot(tx - x, ty - y), 3))

    # --- Switch (timed).
    started = time.monotonic()
    phase.update(value=CompetitionState.PHASE_HANDOVER, sequence=2)
    time.sleep(1.0)
    stop(processes['slam'])
    assets = Path(get_package_share_directory('edgeai_task_coordinator')) / 'test_assets'
    processes['map'] = popen(['ros2', 'launch', 'edgeai_robot_navigation',
                              'dev_map_server.launch.py', f'map:={assets / "midterm.yaml"}'],
                             args.output / 'map_server.log')
    # Relocalize only on the pre-built map (0.02 m cells), not on the last SLAM map.
    step('switch pre-built map received',
         wait(lambda: maps and abs(maps[-1].info.resolution - 0.02) < 1e-6, 30))
    for robot_id in ROBOTS:
        relocalize[robot_id].wait_for_service(timeout_sec=10)
        future = relocalize[robot_id].call_async(Relocalize.Request(
            run_id='run-p1-switch', map_id='midterm', map_revision=1,
            mode=Relocalize.Request.MODE_KNOWN_START_POSE,
            start_pose_id=f'midterm_{robot_id}', timeout=Duration(sec=30)))
        wait(future.done, 60)
        result = future.result()
        ok &= step(f'switch relocalize {robot_id}', result and result.success,
                   message=result.message if result else 'no reply')
    phase.update(value=CompetitionState.PHASE_TASK2A, sequence=3)
    # Dispatch only after every robot acknowledged TASK2A (as P2/P4 must).
    both = wait(lambda: all(ready(r) and acks.get(r, 0) >= 3 for r in ROBOTS), 30)
    switch_s = round(time.monotonic() - started, 1)
    ok &= step('switch both robots ready', both and switch_s <= args.budget,
               wall_s=switch_s, budget_s=args.budget)

    # --- Part 2 on the pre-built map (arena frame).
    if args.rescue:
        part2_started = time.monotonic()
        processes['p4'] = popen(
            ['ros2', 'run', 'edgeai_task_coordinator', 'coordinator', '--ros-args',
             '-p', 'use_sim_time:=true', '-p', f'config_dir:={config}',
             '-p', f'output_dir:={(args.output / "p4").resolve()}'],
            args.output / 'coordinator.log')
        wait(lambda: rescue['freeze'] or rescue['status'].get('status') == 'HALTED'
             or processes['p4'].poll() is not None, args.rescue_timeout)
        freeze = rescue['freeze']
        end = freeze['wall'] if freeze else time.monotonic()
        report['part2_wall_s'] = round(end - part2_started, 1)
        report['total_wall_s'] = round(end - part1_started, 1)
        ok &= step('part2 rescue S1 and C1',
                   bool(freeze and 'completed' in freeze['reason']),
                   reason=freeze['reason'] if freeze else rescue['status'].get('error', 'timeout'),
                   part2_wall_s=report['part2_wall_s'], total_wall_s=report['total_wall_s'])
    else:
        for robot_id in ROBOTS:
            goal = PART2_GOALS[robot_id]
            outcome = navigate(robot_id, goal, f'part2-{robot_id}',
                               ExecuteNavigationGoal.Goal.PURPOSE_TRANSIT)
            tx, ty = gazebo_xy(robot_id)
            ok &= step(f'part2 {robot_id} navigation', outcome == 'SUCCEEDED', result=outcome,
                       true_error_m=round(math.hypot(tx - goal[0], ty - goal[1]), 3))
    report['switch_wall_s'] = switch_s
    report['passed'] = bool(ok)
    executor.shutdown()
    rclpy.try_shutdown()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', required=True, type=Path, help='new report directory')
    parser.add_argument('--budget', type=float, default=60.0, help='switch budget [s]')
    parser.add_argument('--domain', type=int, default=76, help='isolated ROS_DOMAIN_ID')
    parser.add_argument('--gui', action='store_true', help='open Gazebo and RViz')
    parser.add_argument('--rescue', action='store_true',
                        help='run the full Part 2 rescue with P4 after the switch')
    parser.add_argument('--rescue-timeout', type=float, default=420.0,
                        help='Part 2 rescue limit [s wall]')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      GZ_PARTITION=f'p1-switch-check-{os.getpid()}')
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')
    from ament_index_python.packages import get_package_share_directory
    slam_params = os.path.join(get_package_share_directory('edgeai_robot_description'),
                               'config', 'slam_toolbox.yaml')
    processes = {
        'sim': popen(['ros2', 'launch', 'edgeai_robot_bringup', 'midterm_sim.launch.py',
                      'part:=1', f'headless:={str(not args.gui).lower()}',
                      f'rviz:={str(args.gui).lower()}'], args.output / 'launch.log'),
        'slam': popen(['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
                       f'slam_params_file:={slam_params}', 'use_sim_time:=true'],
                      args.output / 'slam.log'),
    }
    try:
        time.sleep(25)
        report = run(args, processes)
    finally:
        for process in processes.values():
            if process.poll() is None:
                stop(process)
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f'passed={report["passed"]}; report in {args.output / "report.json"}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
