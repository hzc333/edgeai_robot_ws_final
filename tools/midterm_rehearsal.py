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
Full midterm rehearsal with every module: robots, phases/SLAM/exploration,
detection and rescue (`task1.launch.py scenario:=midterm`).

This driver only plays the operator, as system_manager's validate_sim does:

1. TASK1. Autonomous exploration and marker detection; robot_1 then drives back
   to the start area and the exploration coordinator requests HANDOVER.
2. HANDOVER. Waits for the frozen map and the frozen target snapshot, then calls
   /edge/finalize_task1.
3. TASK2A. system_manager relocalizes robot_1 (last SLAM pose) and robot_2 (its start
   pose);
   the rescue coordinator rescues S1 and C1 and requests FROZEN.
4. FINISHED, then the audit of the sealed run and the verification summary
   (tools/midterm_report.py).

Detected targets are compared with the marker centres of the midterm rescue config.

Practice arena (--arena DIR from tools/generate_practice_arena.py): the same flow on that
world, Task 1 only (the rescue configuration fits the midterm arena), compared with and
scored against the arena's ground truth (tools/score_task1.py).

Handover and localization check (tools/localization_check.py), in a practice arena or, without
the rescue, in the midterm arena:

    --robots 3|4              robot_3/robot_4 wait outside the arena during Task 1; at
                              HANDOVER the driver places them at handover_robot_N (as the
                              organizers would) and TASK2A starts with all robots
    --check-localization N    in TASK2A every robot drives to N random goals; the report
                              gets the localization and arrival errors against Gazebo
    --task2b stay|reposition  then FROZEN and TASK2B: the robots stay where they stopped
                              (relocalized from their own estimates) or are moved to
                              task2b_robot_N (relocalized there); the check runs again

    pixi run practice --arena runs/practice-arenas/seed-3 --robots 4 --check-localization 3
Run from the repository root after `pixi run build`:

    pixi run bash -c 'source install/setup.bash && python tools/midterm_rehearsal.py --gui'

Live demonstration (`pixi run midterm-demo`): --gui --wait-start --keep-open waits for
Enter before TASK1 starts the 13 min timer and keeps Gazebo and RViz open for the
result verification until Ctrl+C.

Reports go to <runs-root>/<run_id>.rehearsal.json, figures to <run_id>.verification/;
exit status 0 on success.
"""

import argparse
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time


def stop_leftover_gazebo(run_id):
    """Kill this run's Gazebo processes that left the launch's process group.

    With the GUI, `gz sim server` sometimes survives the launch shutdown in its own
    process group, ignores SIGTERM and keeps a CPU busy. The launch gives Gazebo the
    transport partition edgeai-<run_id>, so the environment identifies this run's processes.
    """
    marker = ('GZ_PARTITION=edgeai-' + run_id).encode()

    def leftovers():
        pids = []
        for proc in Path('/proc').iterdir():
            if proc.name.isdigit() and int(proc.name) != os.getpid():
                try:
                    if marker in (proc / 'environ').read_bytes().split(b'\0'):
                        pids.append(int(proc.name))
                except OSError:
                    pass
        return pids

    for sig, pause in ((signal.SIGTERM, 3.0), (signal.SIGKILL, 0.5)):
        pids = leftovers()
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        if pids:
            print(f'stopped leftover Gazebo processes {pids} ({sig.name})', flush=True)
            time.sleep(pause)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--runs-root', default=None,
                        help='default: runs/midterm-rehearsal, with --arena runs/practice')
    parser.add_argument('--arena', type=Path, default=None,
                        help='practice arena directory: run Task 1 there and score it')
    parser.add_argument('--domain', type=int, default=77, help='isolated ROS_DOMAIN_ID')
    parser.add_argument('--gui', action='store_true', help='open Gazebo and RViz')
    parser.add_argument('--task1-timeout', type=float, default=780.0,
                        help='TASK1 limit [s wall]')
    parser.add_argument('--rescue-timeout', type=float, default=420.0,
                        help='TASK2A limit [s wall]')
    parser.add_argument('--wait-start', action='store_true',
                        help='wait for Enter before starting TASK1')
    parser.add_argument('--keep-open', action='store_true',
                        help='keep the simulation running after the run until Ctrl+C')
    parser.add_argument('--debug-on-fail', action='store_true',
                        help='dump robot_2 AMCL pose and TF before teardown on failure')
    parser.add_argument('--robots', type=int, choices=(2, 3, 4), default=2,
                        help='robots in Task 2A; beyond robot_1/robot_2 they join at HANDOVER')
    parser.add_argument('--check-localization', type=int, default=0, metavar='N',
                        help='TASK2A: N random goals per robot, measure localization')
    parser.add_argument('--check-seed', type=int, default=None,
                        help='random seed of the localization check goals')
    parser.add_argument('--split', action='store_true',
                        help='start the robots (with Gazebo) and the edge node as separate '
                             'launches, the robots through $EDGEAI_ROBOT_EXEC '
                             '(tools/edge_link.sh); ROS discovery comes from the environment')
    parser.add_argument('--robots-elsewhere', action='store_true',
                        help='start only the edge node and print the command that starts '
                             'the robots (with Gazebo) on another machine with this run_id')
    parser.add_argument('--task2b', choices=('stay', 'reposition'), default=None,
                        help='after TASK2A and FROZEN, enter TASK2B with the robots where they '
                             'stopped or moved to task2b_robot_N (and check localization again)')
    options = parser.parse_args()
    check = options.check_localization > 0
    if options.robots > 2 and not (options.arena or check):
        parser.error('--robots 3|4 needs --arena or --check-localization (the midterm rescue '
                     'only knows robot_1 and robot_2)')
    split = options.split or options.robots_elsewhere
    if options.split and options.robots_elsewhere:
        parser.error('--split and --robots-elsewhere exclude each other')
    if split and (check or options.robots > 2):
        parser.error('--split and --robots-elsewhere do not support --robots 3|4 or '
                     '--check-localization (they move robots through Gazebo, which runs on '
                     'the robot side)')
    robot_exec = os.environ.get('EDGEAI_ROBOT_EXEC', '').split()
    if options.split and not robot_exec:
        parser.error('--split needs $EDGEAI_ROBOT_EXEC; run it through tools/edge_link.sh')
    if options.task2b and not check:
        parser.error('--task2b needs --check-localization (TASK2B follows the checked TASK2A)')
    robots = [f'robot_{i}' for i in range(1, options.robots + 1)]
    waiting = robots[2:]
    if options.arena:
        options.arena = options.arena.resolve()
        truth = json.loads((options.arena / 'ground_truth.json').read_text())
    options.runs_root = options.runs_root or ('runs/practice' if options.arena
                                              else 'runs/midterm-rehearsal')
    os.environ['ROS_DOMAIN_ID'] = str(options.domain)
    if not split:
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')

    from ament_index_python.packages import get_package_share_directory
    from edgeai_common.config import load_runtime_contract, resolve_parameters
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
    # The launch gives Gazebo this partition; the localization check talks to Gazebo too.
    os.environ['GZ_PARTITION'] = 'edgeai-' + run_id
    log = (root / (run_id + '.launch.log')).open('w')
    rescue = not (options.arena or check)
    command = ([
        'ros2', 'launch', 'edgeai_system_manager', 'task1.launch.py', 'scenario:=midterm',
        'runs_root:=' + str(root), 'run_id:=' + run_id,
        'headless:=' + str(not options.gui).lower(), 'rviz:=' + str(options.gui).lower(),
        'rescue:=' + str(rescue).lower()]
        # ros2 launch rejects an empty value ('waiting_robots:=').
        + (['waiting_robots:=' + ','.join(waiting)] if waiting else [])
        + (['task2b_start_pose_ids:=' + ','.join(f'{r}:task2b_{r}' for r in robots)]
           if options.task2b == 'reposition' else [])
        + (['world:=' + str(options.arena / 'arena.sdf')] if options.arena else []))
    if options.robots_elsewhere:
        processes = [subprocess.Popen(command + ['side:=edge'], stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True)]
        # Paths relative to the repository root: the other machine has its own checkout
        # (a practice arena is regenerated there from the same seed).
        robot_command = ' '.join(
            command[:5] + ['world:=' + os.path.relpath(a[len('world:='):])
                           if a.startswith('world:=') else a
                           for a in command[5:] if not a.startswith('runs_root:=')]
            + ['side:=robots'])
        print(json.dumps({'step': 'start the robots on their machine within 15 min (same '
                          f'ROS_DOMAIN_ID {options.domain} and middleware)',
                          'command': robot_command}), flush=True)
    elif options.split:
        robot_log = (root / (run_id + '.robots.launch.log')).open('w')
        processes = [subprocess.Popen(command + ['side:=edge'], stdout=log,
                                      stderr=subprocess.STDOUT, start_new_session=True),
                     subprocess.Popen(robot_exec + command + ['side:=robots'], stdout=robot_log,
                                      stderr=subprocess.STDOUT, start_new_session=True)]
    else:
        processes = [subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)]
    process = processes[0]
    rclpy.init()
    node = rclpy.create_node('midterm_rehearsal_operator')
    received = {}
    contract = load_runtime_contract()
    for kind, topic, key, qos in [
            (CompetitionState, '/competition/state', 'state', 'control_state'),
            *[(RobotState, f'/{r}/state', r, 'robot_state') for r in robots],
            (TargetArray, '/edge/targets', 'targets', 'snapshot'),
            (String, '/edge/map_metadata', 'map', 'snapshot'),
            (String, '/edge/midterm_rescue/status', 'rescue', 'snapshot')]:
        node.create_subscription(kind, topic, lambda msg, key=key: received.update({key: msg}),
                                 qos_from_contract(contract, qos))
    phase_client = node.create_client(SetCompetitionPhase, '/competition/set_phase')
    finalize_client = node.create_client(FinalizeTask1, '/edge/finalize_task1')
    if options.arena:
        markers = {m['target_id']: [m['x'], m['y']] for m in truth['markers'] if m['target_id']}
    else:
        markers = {t['id']: t['center'] for t in yaml.safe_load(
            (Path(get_package_share_directory('edgeai_task_coordinator')) / 'config'
             / 'midterm.yaml').read_text())['targets']}
    report = {'run_id': run_id, 'success': False, 'times_s': {}}
    if options.arena:
        report['arena'] = {'path': str(options.arena), 'seed': truth.get('seed')}
    rescue_seen = []

    def say(text, **details):
        print(json.dumps({'step': text, **details}, ensure_ascii=False), flush=True)

    def wait(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if 'rescue' in received:
                data = json.loads(received['rescue'].data)
                key = (data.get('status'), data.get('target_id'))
                if not rescue_seen or rescue_seen[-1] != key:
                    rescue_seen.append(key)
                    say('rescue', status=key[0], target=key[1])
            if any(p.poll() is not None for p in processes):
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
        # Entering TASK2A relocalizes the robots one after another.
        timeout = 45 + 35 * len(robots)
        response = call(phase_client, request, timeout)
        while (not response.accepted and response.message == 'another control operation is active'
               and time.monotonic() < deadline):
            time.sleep(0.25)
            response = call(phase_client, request, timeout)
        if not response.accepted:
            raise RuntimeError(f'phase {destination}: {response.message}')

    def entered():
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return True
        return False

    def ready(robot_id):
        state = received.get(robot_id)
        return bool(state and state.localized and state.navigation_available)

    started = time.monotonic()
    try:
        wait(lambda: 'state' in received and ready('robot_1'),
             900 if options.robots_elsewhere else 150, 'SLAM/Nav2 did not become ready')
        if options.wait_start:
            say('ready; press Enter to start TASK1 (the 13 min timer starts)')
            # Keep spinning while waiting so state and rescue status stay current.
            wait(entered, 24 * 3600, 'not started')
            wait(lambda: ready('robot_1'), 30, 'robot_1 no longer ready')
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
        if options.arena and not (check or waiting):
            report['times_s']['handover'] = round(time.monotonic() - handover, 1)
            report['success'] = set(found) >= set(markers)
        elif not rescue:
            report['task1_success'] = set(found) >= set(markers)
            from localization_check import coarsen, GroundTruth, LocalizationCheck, load_map
            from localization_check import merge
            from localization_check import place_robots
            truth_poses = GroundTruth()
            wait(lambda: all(truth_poses.get(r) for r in robots), 20,
                 'no Gazebo poses for the robots')
            if waiting:
                starts = resolve_parameters('development').require('localization.start_poses')
                place_robots(truth_poses, {r: starts['handover_' + r] for r in waiting},
                             robots, wait)
                say('placed the waiting robots', robots=waiting)
            phase(3)
            wait(lambda: received['state'].phase == 3 and all(ready(r) for r in robots), 60,
                 'robots not relocalized for TASK2A')
            report['times_s']['handover'] = round(time.monotonic() - handover, 1)
            say('TASK2A: all robots localized', handover_seconds=report['times_s']['handover'])
            report['success'] = report['task1_success']
            if check:
                map_yaml = (options.arena / 'truth_map.yaml' if options.arena else
                            Path(get_package_share_directory('edgeai_task_coordinator'))
                            / 'test_assets' / 'midterm.yaml')
                # Goals: free in the truth and reachable on the frozen map Nav2 plans on.
                grid = merge(coarsen(load_map(map_yaml)),
                             load_map(root / run_id / 'task1' / 'map.yaml'))

                def run_check(key, label, seed):
                    checker = LocalizationCheck(node, wait, received, truth_poses, robots, grid,
                                                options.check_localization, seed, label=label)
                    started_check = time.monotonic()
                    checker.run(say)
                    report['times_s'][key] = round(time.monotonic() - started_check, 1)
                    report[key] = checker.report()
                    say(key.replace('_', ' '), passed=report[key]['passed'],
                        problems=report[key]['problems'])
                    report['success'] = report['success'] and report[key]['passed']
                run_check('localization_check', 'localization-check', options.check_seed)
            phase(4)
            if options.task2b:
                if options.task2b == 'reposition':
                    # As the organizers would: lift every robot out of the arena first, so
                    # none is set down where another one still stands.
                    starts = resolve_parameters('development').require(
                        'localization.task2b_start_poses')
                    outside = {r: {'x': -0.8, 'y': 0.5 + i, 'yaw': 1.5707963}
                               for i, r in enumerate(robots)}
                    place_robots(truth_poses, outside, robots, wait)
                    place_robots(truth_poses, {r: starts['task2b_' + r] for r in robots},
                                 robots, wait)
                    say('moved the robots to their Task 2B start poses', robots=robots)
                task2b = time.monotonic()
                phase(5)
                wait(lambda: received['state'].phase == 5 and all(ready(r) for r in robots), 60,
                     'robots not relocalized for TASK2B')
                report['times_s']['task2b_relocalization'] = round(time.monotonic() - task2b, 1)
                say('TASK2B: all robots localized', mode=options.task2b,
                    seconds=report['times_s']['task2b_relocalization'])
                run_check('localization_check_task2b', 'localization-check-task2b',
                          None if options.check_seed is None else options.check_seed + 1000)
            phase(6)
            report['audit'] = audit(root / run_id, Path.cwd() / 'schemas', require_finished=True)
        else:
            phase(3)
            # The coordinator may already have requested FROZEN (phase 4) when it fails closed.
            wait(lambda: received['state'].phase in (3, 4) and ready('robot_1')
                 and ready('robot_2'), 60, 'robots not relocalized for TASK2A')
            report['times_s']['handover'] = round(time.monotonic() - handover, 1)
            say('TASK2A: rescue', handover_seconds=report['times_s']['handover'])
            rescue = time.monotonic()
            wait(lambda: received['state'].phase == 4, options.rescue_timeout,
                 'the rescue did not finish')
            report['rescue_reason'] = received['state'].reason
            report['times_s']['task2a'] = round(time.monotonic() - rescue, 1)
            report['times_s']['total'] = round(time.monotonic() - task1, 1)
            rescued = bool(rescue_seen and rescue_seen[-1][0] == 'DONE')
            say('FROZEN', reason=report['rescue_reason'], rescue_done=rescued,
                seconds=report['times_s']['task2a'], total_seconds=report['times_s']['total'])
            phase(6)
            report['audit'] = audit(root / run_id, Path.cwd() / 'schemas', require_finished=True)
            report['success'] = rescued and set(found) >= set(markers)
    except (Exception, KeyboardInterrupt) as error:
        report['error'] = str(error)
        report['success'] = False   # the checks may already have set it
        report['robots'] = {
            r: {'localized': m.localized, 'navigation_available': m.navigation_available,
                'mode': m.mode, 'xy': [round(m.pose.pose.position.x, 2),
                                       round(m.pose.pose.position.y, 2)]}
            for r, m in ((r, received.get(r)) for r in robots) if m}
        if 'rescue' in received:
            report['rescue_last_status'] = json.loads(received['rescue'].data)
        say('failed', error=str(error), robots=report['robots'],
            rescue_error=report.get('rescue_last_status', {}).get('error'))
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
        report['rescue_status_sequence'] = rescue_seen
        report['wall_s'] = round(time.monotonic() - started, 1)
        atomic_json(root / (run_id + '.rehearsal.json'), report)
        if not report['success']:
            try:
                phase(7)
            except Exception:
                pass
        if options.arena and (root / run_id / 'task1' / 'targets.json').exists():
            subprocess.run([sys.executable, str(Path(__file__).with_name('score_task1.py')),
                            str(root / run_id), str(options.arena)])
        elif (root / run_id / 'task1' / 'map.yaml').exists():
            subprocess.run([sys.executable, str(Path(__file__).with_name('midterm_report.py')),
                            str(root / run_id)])
        if options.keep_open and process.poll() is None:
            say('run over; Gazebo and RViz stay open for the verification, Ctrl+C closes them')
            try:
                while process.poll() is None:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                pass
        for launched in processes:
            if launched.poll() is None:
                os.killpg(launched.pid, signal.SIGINT)
        for launched in processes:
            try:
                launched.wait(timeout=25)
            except subprocess.TimeoutExpired:
                pass
        # gz's Ruby launcher may exit while its server remains in our process group.
        for launched in processes:
            try:
                os.killpg(launched.pid, signal.SIGTERM)
                time.sleep(0.5)
                os.killpg(launched.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        stop_leftover_gazebo(run_id)
        log.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f'passed={report["success"]}; report in {root / (run_id + ".rehearsal.json")}')
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
