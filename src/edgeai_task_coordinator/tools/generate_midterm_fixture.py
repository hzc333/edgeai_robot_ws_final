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

"""Generate P4-only Part 2 world/map fixtures from official arena dimensions."""

import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml


# name, center_x, center_y, width, depth; all metres in the official arena frame.
BOXES = [
    ('south', 2.5, 0.0, 5.1, 0.1), ('north', 2.5, 5.0, 5.1, 0.1),
    ('west', 0.0, 2.5, 0.1, 5.1), ('east', 5.0, 2.5, 0.1, 5.1),
    ('W1', 1.1, 3.1, 2.2, 0.1), ('W1b', 2.2, 3.35, 0.1, 0.5),
    ('W2', 3.4, 1.4, 0.1, 2.8), ('O2', 2.2, 2.0, 0.6, 1.0),
    ('O3', 4.4, 3.6, 0.8, 0.6),
    ('S1_stand', 1.0, 4.1, 0.18, 0.16), ('C1_stand', 4.1, 1.0, 0.18, 0.16),
]
# Marker stand column (m): footprint inside the stand map box, top above the LiDAR plane.
STAND_WIDTH, STAND_DEPTH, STAND_HEIGHT = 0.12, 0.08, 0.25


def sub(parent, tag, text=None, **attributes):
    item = ET.SubElement(parent, tag, attributes)
    if text is not None:
        item.text = str(text)
    return item


def box(link, name, width, depth, height, *, pose=None, collision=True, color=None):
    for kind in (('collision', 'visual') if collision else ('visual',)):
        element = sub(link, kind, name=name + '_' + kind)
        if pose:
            sub(element, 'pose', pose)
        geometry = sub(element, 'geometry')
        sub(sub(geometry, 'box'), 'size', f'{width} {depth} {height}')
        if kind == 'visual' and color:
            material = sub(element, 'material')
            sub(material, 'ambient', color)
            sub(material, 'diffuse', color)


def generate(output):
    output.mkdir(parents=True, exist_ok=True)
    root = ET.Element('sdf', version='1.9')
    world = sub(root, 'world', name='empty')  # reuse P1's world services/spawn interfaces
    physics = sub(world, 'physics', name='physics', type='ignored')
    sub(physics, 'max_step_size', 0.002)
    sub(physics, 'real_time_factor', 1.0)
    for library, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                          ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu'),
                          ('contact', 'Contact'), ('sensors', 'Sensors')]:
        plugin = sub(world, 'plugin', filename=f'gz-sim-{library}-system',
                     name=f'gz::sim::systems::{name}')
        if library == 'sensors':
            sub(plugin, 'render_engine', 'ogre2')
    sub(world, 'plugin', filename='edgeai_p4_contacts', name='edgeai::Contacts')
    floor = sub(world, 'model', name='ground_plane')
    sub(floor, 'static', 'true')
    floor_link = sub(floor, 'link', name='link')
    for kind in ('collision', 'visual'):
        plane = sub(sub(sub(floor_link, kind, name=kind), 'geometry'), 'plane')
        sub(plane, 'normal', '0 0 1')
        sub(plane, 'size', '100 100')
    # Markers are real 4x4_50 patterns made of planar black/white tiles; this
    # Part 2 fixture does not claim Part 1 camera/detection acceptance.
    import cv2
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for name, x, y, width, depth in BOXES:
        stand = name.endswith('stand')
        height = 0.03 if stand else 0.5
        model = sub(world, 'model', name=name)
        sub(model, 'static', 'true')
        sub(model, 'pose', f'{x} {y} {height / 2} 0 0 0')
        link = sub(model, 'link', name='link')
        if stand:
            # The stand body is a column tall enough for the 0.155 m LiDAR plane (a low
            # base would be invisible to SLAM and the collision monitor), set behind the
            # marker board so it does not hide the marker from the low camera. Its
            # footprint stays inside the (width x depth) map box above.
            behind = STAND_DEPTH / 2 * (1 if name.startswith('S1') else -1)
            box(link, 'body', STAND_WIDTH, STAND_DEPTH, STAND_HEIGHT,
                pose=f'0 {behind} {STAND_HEIGHT / 2 - height / 2} 0 0 0',
                color='0.35 0.35 0.35 1')
        else:
            box(link, 'body', width, depth, height, color='0.35 0.35 0.35 1')
        sensor = sub(link, 'sensor', name='contact', type='contact')
        sub(sensor, 'always_on', 'true')
        sub(sensor, 'update_rate', 30)
        sub(sensor, 'topic', f'/p4_contacts/{name}')
        contact = sub(sensor, 'contact')
        sub(contact, 'collision', 'body_collision')
        sub(contact, 'topic', f'/p4_contacts/{name}')
        if stand:
            box(link, 'pole', 0.025, 0.025, 0.32,
                pose='0 0 0.16 0 0 0', color='0.15 0.15 0.15 1')
            box(link, 'board', 0.20, 0.012, 0.20,
                pose='0 0 0.335 0 0 0', color='1 1 1 1')
            marker_id = 1 if name.startswith('S1') else 2
            pattern = cv2.aruco.generateImageMarker(dictionary, marker_id, 6)
            face = -0.0065 if marker_id == 1 else 0.0065
            for row in range(6):
                for col in range(6):
                    # Viewed from south, screen right is +x; from north it is -x.
                    px = (col - 2.5) * 0.16 / 6 * (1 if marker_id == 1 else -1)
                    pz = 0.335 + (2.5 - row) * 0.16 / 6
                    color = '0 0 0 1' if pattern[row, col] == 0 else '1 1 1 1'
                    box(link, f'pixel_{row}_{col}', 0.16 / 6, 0.001, 0.16 / 6,
                        pose=f'{px} {face} {pz} 0 0 0', collision=False, color=color)
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 10 0 0 0')
    sub(light, 'diffuse', '0.8 0.8 0.8 1')
    sub(light, 'direction', '-0.3 0.2 -1')
    ET.indent(root)
    ET.ElementTree(root).write(output / 'midterm.sdf', encoding='utf-8', xml_declaration=True)
    resolution, origin, size = 0.02, -0.2, 270
    pixels = bytearray()
    for row in reversed(range(size)):
        y = origin + (row + 0.5) * resolution
        for col in range(size):
            x = origin + (col + 0.5) * resolution
            occupied = any(abs(x - bx) <= width / 2 and abs(y - by) <= depth / 2
                           for _, bx, by, width, depth in BOXES)
            pixels.append(0 if occupied else 254)
    (output / 'midterm.pgm').write_bytes(f'P5\n{size} {size}\n255\n'.encode() + pixels)
    (output / 'midterm.yaml').write_text(yaml.safe_dump({
        'image': 'midterm.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': [origin, origin, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False))
    (output / 'geometry.json').write_text(__import__('json').dumps(BOXES, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    generate(parser.parse_args().output)
