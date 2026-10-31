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

"""Reproduce Part 2 with real Gazebo/Nav2/AMCL and an explicit P2 test stand-in."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid

from generate_midterm_fixture import BOXES
from simulation_driver import RemoteCoordinator


def polygon(x, y, yaw, vertices):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in vertices]


def overlap(first, second):
    """Separating-axis test on ground-truth footprint envelopes."""
    for poly in (first, second):
        for a, b in zip(poly, poly[1:] + poly[:1]):
            axis = (a[1] - b[1], b[0] - a[0])
            one = [p[0] * axis[0] + p[1] * axis[1] for p in first]
            two = [p[0] * axis[0] + p[1] * axis[1] for p in second]
            if min(max(one), max(two)) - max(min(one), min(two)) <= 1e-6:
                return False
    return True


def run(args):
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    package = Path(__file__).resolve().parents[1]
    sources = [*package.glob('edgeai_task_coordinator/*.py'),
               *package.glob('config/*.yaml'), *package.glob('launch/*.py'),
               *package.glob('tools/*.py'), *package.glob('gazebo_plugin/*.cc')]
    (output / 'source_manifest.json').write_text(json.dumps({
        str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(sources)}, indent=2))
    os.environ.update({
        'ROS_DOMAIN_ID': str(args.domain), 'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        'ROS_LOG_DIR': str(output / 'ros-log'),
        'GZ_PARTITION': 'p4-' + uuid.uuid4().hex, 'GZ_IP': '127.0.0.1',
        'GZ_HOMEDIR': str(output / 'gz-home'),
        'CYCLONEDDS_URI': ('<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>200'
                           '</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'),
    })
    nvidia_vendor = Path('/usr/share/glvnd/egl_vendor.d/10_nvidia.json')
    if nvidia_vendor.exists() and Path('/dev/nvidia0').exists():
        # Mesa can enumerate an NVIDIA DRM device and crash before falling back
        # to software. Keep this selection local to this validation process.
        os.environ.setdefault('__EGL_VENDOR_LIBRARY_FILENAMES', str(nvidia_vendor))
        os.environ.setdefault('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    (output / 'test_environment.json').write_text(json.dumps({
        key: os.environ.get(key) for key in (
            'ROS_DOMAIN_ID', 'GZ_PARTITION', '__EGL_VENDOR_LIBRARY_FILENAMES',
            '__GLX_VENDOR_LIBRARY_NAME', 'GALLIUM_DRIVER', 'LIBGL_ALWAYS_SOFTWARE')}, indent=2))
    # Imports follow domain/partition setup: no test participant escapes into a live run.
    from edgeai_common.config import find_config_dir, load_runtime_contract
    from edgeai_common.qos import qos_from_contract
    from edgeai_robot_interfaces.msg import CompetitionState, RobotPhaseAck
    from edgeai_robot_interfaces.srv import Relocalize, SetCompetitionPhase
    from gz.msgs10.contacts_pb2 import Contacts
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node as GzNode
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions
    import yaml

    from edgeai_task_coordinator.verification import DwellVerifier
    from edgeai_task_coordinator.model import Robot

    config_dir = output / 'p1-config'
    config_dir.mkdir()
    source = find_config_dir()
    for name in ('competition_parameters.yaml', 'interface_runtime.yaml'):
        shutil.copyfile(source / name, config_dir / name)
    parameters = yaml.safe_load((config_dir / 'competition_parameters.yaml').read_text())
    layouts = {
        'nominal': [(0.55, 0.5, 0.0), (1.20, 0.5, 0.0)],
        'swapped': [(1.20, 0.5, 0.0), (0.55, 0.5, 0.0)],
        'staggered': [(0.55, 0.5, math.pi / 2), (1.20, 0.75, 0.0)],
        # Mirrors P1's config/sim/robots_midterm.yaml (side by side, facing north).
        'midterm': [(0.40, 0.5, math.pi / 2), (1.05, 0.5, math.pi / 2)],
    }
    starts = dict(zip(('robot_1', 'robot_2'), layouts[args.start_layout]))
    parameters['development']['localization']['start_poses'] = {
        f'p4_{rid}_start': dict(zip(('x', 'y', 'yaw'), pose)) for rid, pose in starts.items()}
    (config_dir / 'competition_parameters.yaml').write_text(yaml.safe_dump(parameters))
    launch_log = (output / 'launch.log').open('w')
    launch_command = [
        'ros2', 'launch', 'edgeai_task_coordinator', 'development_sim.launch.py',
        f'config_dir:={config_dir}', f'headless:={str(not args.gui).lower()}',
        *(f'{rid}_start:=' + ','.join(str(v) for v in pose) for rid, pose in starts.items()),
    ]
    process = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    control = Node('p4_simulation_control', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    coordinator = RemoteCoordinator(control, load_runtime_contract(config_dir), config_dir, output)
    executor = SingleThreadedExecutor()
    executor.add_node(control)
    qos = qos_from_contract(load_runtime_contract(config_dir), 'control_state')
    phase_pub = control.create_publisher(CompetitionState, '/competition/state', qos)
    run_id, map_id = 'p4-sim-' + uuid.uuid4().hex, 'official-midterm-fixture-v1'
    phase, sequence = CompetitionState.PHASE_HANDOVER, 1
    acks = {}
    control.create_subscription(
        RobotPhaseAck, '/robot_1/phase_ack', lambda m: acks.update({m.robot_id: m}), 1)
    control.create_subscription(
        RobotPhaseAck, '/robot_2/phase_ack', lambda m: acks.update({m.robot_id: m}), 1)

    def publish_phase():
        msg = CompetitionState(run_id=run_id, map_id=map_id, map_revision=1,
                               phase=phase, phase_sequence=sequence)
        msg.header.stamp = control.get_clock().now().to_msg()
        phase_pub.publish(msg)

    finish_requests = []

    def set_phase(request, response):
        nonlocal phase, sequence
        response.accepted = (request.run_id == run_id
                             and request.requested_phase == CompetitionState.PHASE_FROZEN)
        if response.accepted:
            finish_requests.append({'sim_time': control.get_clock().now().nanoseconds / 1e9,
                                    'reason': request.reason})
            if phase != CompetitionState.PHASE_FROZEN:
                phase, sequence = CompetitionState.PHASE_FROZEN, sequence + 1
            publish_phase()
        response.current_phase, response.phase_sequence = phase, sequence
        return response

    control.create_service(SetCompetitionPhase, '/competition/set_phase', set_phase)
    # A steady timer maintains P1's wall-clock edge-connection watchdog even at low RTF.
    control.create_timer(0.2, publish_phase, clock=coordinator.steady_clock)
    truth = {}
    contacts = []
    truth_lock = threading.Lock()
    transport = GzNode()
    observed_truth = set()
    observed_contacts = set()

    def on_pose(msg):
        with truth_lock:
            for pose in msg.pose:
                if pose.name in ('robot_1', 'robot_2'):
                    q = pose.orientation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    truth[pose.name] = (msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                                        pose.position.x,
                                        pose.position.y, yaw)
                    observed_truth.add(pose.name)

    def on_contacts(msg, name):
        observed_contacts.add(name)
        for contact in msg.contact:
            names = contact.collision1.name + ' ' + contact.collision2.name
            if 'robot_' in names:
                with truth_lock:
                    contacts.append({'sim_time': (msg.header.stamp.sec
                                                  + msg.header.stamp.nsec * 1e-9),
                                     'obstacle': name, 'collisions': names})

    transport.subscribe(Pose_V, '/world/empty/dynamic_pose/info', on_pose)
    for name, *_ in BOXES:
        transport.subscribe(Contacts, f'/p4_contacts/{name}',
                            lambda msg, n=name: on_contacts(msg, n))

    clients = {rid: control.create_client(Relocalize, f'/{rid}/relocalize')
               for rid in ('robot_1', 'robot_2')}
    localization = {}
    localization_sent = {}
    truth_verifiers = {t.target_id: DwellVerifier() for t in coordinator.config.targets}
    truth_completed = {}
    overlaps = []
    # Same conservative footprint as P1 costmaps; an envelope overlap is not
    # silently described as a physics contact.
    footprint = [(0.115, 0.175), (0.115, -0.175), (-0.315, -0.175), (-0.315, 0.175)]
    distances = (output / 'ground_truth.jsonl').open('w')
    started = time.monotonic()
    task_started = None
    last_progress = started - 10
    last_truth_stamp = None
    success = False
    recovery_injections = []
    resume_at = None
    freeze_injected = False
    restart_injected = False
    try:
        process = subprocess.Popen(launch_command, stdout=launch_log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        while time.monotonic() - started < args.timeout_wall:
            executor.spin_once(timeout_sec=0.05)
            wall = time.monotonic()
            sim = control.get_clock().now().nanoseconds / 1e9
            if coordinator.process.poll() is not None:
                raise RuntimeError('P4 process exited; inspect coordinator.log')
            if process.poll() is not None:
                raise RuntimeError('simulation launch exited; inspect launch.log')
            if wall - started > 30 and sim <= 0:
                raise RuntimeError('no simulation clock after 30s; inspect launch.log')
            if any(not f.done() and wall - localization_sent[rid] > 80
                   for rid, f in localization.items()):
                raise RuntimeError('localization service response timeout')
            if phase == CompetitionState.PHASE_HANDOVER and sim > 1 and coordinator.grid:
                for rid, client in clients.items():
                    if rid not in localization and client.service_is_ready():
                        request = Relocalize.Request(
                            run_id=run_id, map_id=map_id, map_revision=1,
                            mode=Relocalize.Request.MODE_KNOWN_START_POSE,
                            start_pose_id=f'p4_{rid}_start')
                        request.timeout.sec = 45
                        localization[rid] = client.call_async(request)
                        localization_sent[rid] = wall
                if len(localization) == 2 and all(f.done() for f in localization.values()):
                    results = {r: f.result() for r, f in localization.items()}
                    if not all(r.success for r in results.values()):
                        raise RuntimeError('localization failed: ' + repr(
                            {r: m.message for r, m in results.items()}))
                    phase, sequence = CompetitionState.PHASE_TASK2A, 2
                    task_started = wall
                    publish_phase()
            if args.exercise_recovery and coordinator.engine:
                state = coordinator.engine.snapshot()
                if (not freeze_injected and state.get('target_id') == 'S1'
                        and state.get('dwell_elapsed_s', 0) >= 2):
                    freeze_injected = True
                    phase, sequence = CompetitionState.PHASE_FROZEN, sequence + 1
                    publish_phase()
                    resume_at = wall + 2
                    recovery_injections.append({'kind': 'freeze_during_s1', 'sim_time': sim})
                if resume_at is not None and wall >= resume_at:
                    phase, sequence = CompetitionState.PHASE_TASK2A, sequence + 1
                    publish_phase()
                    resume_at = None
                if (not restart_injected and state.get('target_id') == 'C1'
                        and state.get('dwell_elapsed_s', 0) >= 2):
                    restart_injected = True
                    recovery_injections.append(dict(
                        kind='sigkill_restart_during_c1', sim_time=sim,
                        **coordinator.crash_restart()))
            with truth_lock:
                positions = dict(truth)
            if len(positions) == 2:
                stamp = min(p[0] for p in positions.values())
                if stamp != last_truth_stamp:
                    last_truth_stamp = stamp
                    sample = {'sim_time': stamp, 'poses': positions}
                    polygons = {r: polygon(p[1], p[2], p[3], footprint)
                                for r, p in positions.items()}
                    if overlap(polygons['robot_1'], polygons['robot_2']):
                        overlaps.append({'sim_time': stamp, 'with': 'robot_peer'})
                    for rid, poly in polygons.items():
                        for name, x, y, width, depth in BOXES:
                            obstacle = polygon(x, y, 0, [
                                (-width / 2, -depth / 2), (width / 2, -depth / 2),
                                (width / 2, depth / 2), (-width / 2, depth / 2)])
                            if overlap(poly, obstacle):
                                overlaps.append({'sim_time': stamp, 'robot': rid, 'with': name})
                    engine = coordinator.engine
                    if engine and engine.target and engine.assignment:
                        target = engine.target
                        chosen = [Robot(rid, positions[rid][0], positions[rid][1],
                                        positions[rid][2]) for rid in engine.assignment]
                        owned = engine.status == 'ACTIVE' and all(
                            s['state'] == 'active' for s in engine.slots.values())
                        verifier = truth_verifiers[target.target_id]
                        done, _, measured = verifier.update(
                            stamp, chosen, target, coordinator.config.settings, valid=owned)
                        sample.update({'target_id': target.target_id, 'distances_m': measured,
                                       'truth_dwell_s': verifier.elapsed})
                        if done and target.target_id not in truth_completed:
                            truth_completed[target.target_id] = {
                                'sim_time': stamp, 'dwell_s': verifier.elapsed}
                    distances.write(json.dumps(sample) + '\n')
                    distances.flush()
            if wall - last_progress >= 10:
                status = (coordinator.engine.snapshot() if coordinator.engine
                          else {'status': 'LOCALIZING'})
                print(json.dumps({'wall_elapsed_s': round(wall - started, 1),
                                  'sim_time': round(sim, 2),
                                  **{k: status.get(k) for k in (
                                      'status', 'target_id', 'dwell_elapsed_s',
                                      'attempts', 'failure', 'collision_status',
                                      'recovery_gate', 'dispatch_gate')}}), flush=True)
                last_progress = wall
            if coordinator.engine and coordinator.engine.status in ('DONE', 'HALTED'):
                success = (coordinator.engine.status == 'DONE'
                           and coordinator.data.get('finish_request') == 'ACCEPTED')
                if success or coordinator.engine.status == 'HALTED':
                    break
        if phase != CompetitionState.PHASE_FROZEN:
            phase, sequence = CompetitionState.PHASE_FROZEN, sequence + 1
        publish_phase()
        end = time.monotonic() + 4
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        safe = all(r in acks and acks[r].phase_sequence == sequence and acks[r].safe
                   for r in clients)
        events = []
        if coordinator.journal:
            events = [json.loads(line) for line in
                      (coordinator.journal.directory / 'events.jsonl').read_text().splitlines()]
        timing_checked = all(
            e['target_id'] in truth_completed
            and truth_completed[e['target_id']]['sim_time'] <= e['sim_time']
            for e in events if e['type'] == 'COMPLETED')
        contact_topics = [name for name, *_ in BOXES
                          if f'/p4_contacts/{name}' in transport.topic_list()]
        report = {
            'scope': 'P4 Part 2 only; P2 phase/localization stand-in, P1 real Nav2/AMCL/Gazebo',
            'run_mode': 'development_with_official_midterm_rescue_rules',
            'p1_transit_goal_timeout_wall_s': coordinator.config.settings.executor_timeout_wall_s,
            'run_id': run_id, 'map_id': map_id, 'robot_footprint_source': 'P1 nav2_sim.yaml',
            'start_layout': args.start_layout, 'start_poses': starts,
            'p4_status': coordinator.engine.snapshot() if coordinator.engine else None,
            'error': coordinator.last_error, 'truth_completed': truth_completed,
            'wall_elapsed_s': time.monotonic() - started,
            'part2_wall_elapsed_s': time.monotonic() - task_started if task_started else None,
            'ground_truth_observed': sorted(observed_truth),
            'physics_contact_sensor_topics_observed': ['/edgeai/physics_contacts']
            if coordinator.data.get('collision', {}).get('state') == 'CLEAR' else [],
            'collision_observer': coordinator.data.get('collision', {}),
            'recovery_injections': recovery_injections, 'finish_requests': finish_requests,
            'physics_contact_sensor_topics_advertised': sorted(contact_topics),
            'physics_contacts': contacts, 'footprint_envelope_overlaps': overlaps,
            'completion_after_truth_dwell_confirmed': timing_checked,
            'freeze_safe_confirmed': safe,
            'passed': (success and set(truth_completed) == {'S1', 'C1'} and timing_checked
                       and not coordinator.last_error and not contacts and not overlaps and safe
                       and coordinator.data.get('collision', {}).get('state') == 'CLEAR'
                       and not coordinator.data.get('collision', {}).get('ever_contact', True)
                       and (not args.exercise_recovery or (freeze_injected and restart_injected))),
            'limitations': ['does not verify Part 1 detection/exploration',
                            'does not verify shared 13-minute whole-team timer',
                            'team production P2 phase/log pipeline still needs integration'],
        }
        (output / 'report.json').write_text(json.dumps(report, indent=2))
        brief = {k: v for k, v in report.items()
                 if k not in ('physics_contacts', 'footprint_envelope_overlaps')}
        print(json.dumps(brief, indent=2))
        return 0 if report['passed'] else 1
    except Exception as error:
        failure = {'passed': False, 'error': f'{type(error).__name__}: {error}',
                   'run_id': run_id, 'scope': 'P4 simulation validation startup/runtime'}
        (output / 'report.json').write_text(json.dumps(failure, indent=2))
        print(json.dumps(failure), flush=True)
        return 1
    finally:
        coordinator.close()
        if process is not None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        control.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        distances.close()
        launch_log.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='new run directory')
    parser.add_argument('--domain', type=int, default=73)
    parser.add_argument('--timeout-wall', type=float, default=900)
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--exercise-recovery', action='store_true')
    parser.add_argument('--start-layout', choices=['nominal', 'swapped', 'staggered', 'midterm'],
                        default='nominal')
    raise SystemExit(run(parser.parse_args()))
