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

"""Exercise real Gazebo contacts: clear, wall, robot peer and marker stand."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET


def run(output):
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ.update(GZ_PARTITION='edgeai-contact-probe-' + uuid.uuid4().hex, GZ_IP='127.0.0.1')
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    from edgeai_task_coordinator.collision import CollisionState
    from edgeai_task_coordinator.physics import ContactFeed, plugin_path

    world = ET.fromstring(
        '<sdf version="1.9"><world name="contact_probe">'
        '<physics name="default" type="ignored">'
        '<max_step_size>0.002</max_step_size><real_time_factor>1</real_time_factor>'
        '</physics></world></sdf>')
    arena = world.find('world')
    for filename, name in [('gz-sim-physics-system', 'gz::sim::systems::Physics'),
                           ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
                           (plugin_path(), 'edgeai::Contacts')]:
        ET.SubElement(arena, 'plugin', filename=filename, name=name)
    for name, pose, size, static in [
            ('ground_plane', '0 0 -0.05 0 0 0', '10 10 0.1', True),
            ('wall', '3 2 0.5 0 0 0', '0.1 4 1', True),
            ('stand', '1 3 0.35 0 0 0', '0.18 0.16 0.7', True),
            ('robot_1', '1 1 0.11 0 0 0', '0.4 0.3 0.2', False),
            ('robot_2', '2 1 0.11 0 0 0', '0.4 0.3 0.2', False)]:
        model = ET.SubElement(arena, 'model', name=name)
        ET.SubElement(model, 'pose').text = pose
        ET.SubElement(model, 'static').text = str(static).lower()
        link = ET.SubElement(model, 'link', name='body')
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '1'
        inertia = ET.SubElement(inertial, 'inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            ET.SubElement(inertia, axis).text = '0.02'
        geometry = ET.SubElement(ET.SubElement(link, 'collision', name='solid'), 'geometry')
        ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = size
    ET.ElementTree(world).write(output / 'world.sdf')
    feed = ContactFeed()
    transport = Node()
    state = CollisionState(('robot_1', 'robot_2'), require_replay=True)
    log = (output / 'gazebo.log').open('w')
    process = subprocess.Popen(['gz', 'sim', '-r', '-s', str(output / 'world.sdf')],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    samples = (output / 'contacts.jsonl').open('w')
    checks = []

    def collect(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for frame in feed.drain():
                state.ingest(*frame)
                snapshot = state.snapshot(frame[0], time.monotonic())
                samples.write(json.dumps(snapshot) + '\n')
                if predicate(snapshot):
                    samples.flush()
                    return snapshot
            if process.poll() is not None:
                raise RuntimeError('Gazebo exited; inspect gazebo.log')
            time.sleep(0.02)
        raise TimeoutError('contact observer condition not met')

    def move(x, y):
        request = Pose(name='robot_1')
        request.position.x, request.position.y, request.position.z = x, y, 0.11
        request.orientation.w = 1.0
        ok, response = transport.request('/world/contact_probe/set_pose', request, Pose, Boolean,
                                         2000)
        if not ok or not response.data:
            raise RuntimeError('set_pose failed')

    try:
        checks.append({'case': 'clear', 'snapshot': collect(lambda s: s['state'] == 'CLEAR')})
        for name, x, y, other in [('wall', 2.84, 1, 'wall'),
                                  ('peer', 1.65, 1, 'robot_2'),
                                  ('stand', 1, 2.80, 'stand')]:
            before = state.count
            move(x, y)
            hit = collect(lambda s: s['count'] > before and any(
                other in item for item in (s['last'] or {}).get('pair', [])))
            checks.append({'case': name, 'snapshot': hit})
            move(1, 1)
            clear = collect(lambda s: s['state'] == 'CLEAR')
            assert clear['ever_contact'] and clear['count'] >= before + 1
        # No consumer is alive while the contact occurs and ends. The Gazebo
        # observer must replay the missed episode to the replacement consumer.
        previous = state.count
        saved = state.history()
        feed.close()
        move(2.84, 1)
        time.sleep(0.3)
        move(1, 1)
        time.sleep(0.2)
        feed = ContactFeed()
        state = CollisionState(('robot_1', 'robot_2'), require_replay=True)
        state.restore(saved)
        replay = collect(lambda s: s['state'] == 'CLEAR' and s['count'] > previous)
        checks.append({'case': 'contact_during_consumer_restart', 'snapshot': replay})
        previous = state.count
        persisted = CollisionState(state.robots)
        persisted.restore(state.history())
        assert persisted.count == previous
        assert persisted.snapshot(0, time.monotonic())['state'] == 'UNKNOWN'
        report = {'passed': True, 'checks': checks, 'history': state.history()}
    except Exception as error:
        report = {'passed': False, 'checks': checks, 'error': str(error)}
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        feed.close()
        samples.close()
        log.close()
    (output / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    raise SystemExit(run(parser.parse_args().output))
