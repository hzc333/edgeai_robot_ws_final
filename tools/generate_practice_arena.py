#!/usr/bin/env python3
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
Random practice arenas for Task 1: rooms, doorways, obstacles and ArUco marker stands.

The final arena is unknown (rulebook 3.1: about 5 m x 5 m with rooms, corridors, walls,
simple obstacles, rescue targets and a start zone), so exploration and detection are
practised on layouts nobody has tuned the code for. Each seed gives one layout in the
arena frame of the midterm scenario (origin at the lower-left corner, start area
x 0-1.4 m, y 0-1.0 m), so the robots' start poses (config/sim/robots_midterm.yaml) apply.
Every layout is checked: the robot (inflated footprint) can reach every room and a
viewpoint in front of every marker from the start area, and the handover poses of robots
that join for Task 2A (localization.start_poses handover_robot_N) leave room to turn in place.

Output directory:

  arena.sdf           Gazebo world (task1.launch.py scenario:=midterm world:=...)
  ground_truth.json   walls, obstacles and marker poses, for scoring only
                      (tools/score_task1.py); never an input to the robots
  truth_map.pgm/.yaml occupancy of the true geometry at the LiDAR height

    python tools/generate_practice_arena.py --seed 7 runs/practice-arenas/seed-7

Markers use the IDs of the development aruco.id_mapping (config/competition_parameters.yaml);
--distractors adds markers with IDs outside the mapping, which must never become targets.
"""

import argparse
import json
import math
from pathlib import Path
import random
from xml.etree import ElementTree as ET

import yaml

REPOSITORY = Path(__file__).resolve().parents[1]
ARENA = 5.0                                  # side of the square arena [m]
WALL_THICKNESS, WALL_HEIGHT = 0.1, 0.5
# No interior geometry here: the start area (x 0-1.4, y 0-1.0) plus room to leave it.
START_KEEP_CLEAR = (0.0, 0.0, 1.7, 1.4)      # x min, y min, x max, y max [m]
ROOM_MIN = 1.3                               # smallest room side [m]
DOOR_WIDTH = (0.8, 1.0)                      # doorway width range [m]
OBSTACLE_SIZE = (0.3, 0.7)                   # obstacle side range [m]
# Free gap kept between obstacles or free-standing stands and other geometry: wide enough
# to drive through (the footprint is 0.35 m wide, a turn in place sweeps 0.36 m).
PASSAGE_M = 0.75
# Reachability: occupied cells grown by the footprint's inscribed radius plus a margin.
ROBOT_CLEARANCE_M = 0.25
VIEW_DISTANCE_M = 0.7                        # viewpoint in front of a marker [m]
MARKER_SPACING_M = 1.0
# Free circle around each handover pose: turn-in-place sweep (0.36 m) plus a margin.
HANDOVER_CLEAR_M = 0.41
GRID_M = 0.05
# Marker stand, as in the midterm arena: a column behind a 0.20 m board that carries a
# 0.16 m marker (4x4 cells plus the black border, 6x6 tiles) centred 0.35 m above ground.
STAND_WIDTH, STAND_DEPTH, STAND_HEIGHT = 0.12, 0.08, 0.25
STAND_RADIUS = 0.13                          # bounding circle of board and column [m]
BASE_HEIGHT = 0.03
BOARD_SIDE, BOARD_THICKNESS, BOARD_Z = 0.20, 0.012, 0.335
MARKER_SIDE = 0.16


class Box:
    """Axis-aligned wall or obstacle: centre and full extents [m]."""

    def __init__(self, name, kind, x, y, width, depth):
        self.name, self.kind = name, kind
        self.x, self.y, self.width, self.depth = x, y, width, depth

    def gap(self, x, y, width=0.0, depth=0.0):
        """Free distance between this box and another axis-aligned box (0 if they touch)."""
        dx = max(0.0, abs(x - self.x) - (width + self.width) / 2)
        dy = max(0.0, abs(y - self.y) - (depth + self.depth) / 2)
        return math.hypot(dx, dy)

    def contains(self, x, y, margin=0.0):
        return (abs(x - self.x) <= self.width / 2 + margin
                and abs(y - self.y) <= self.depth / 2 + margin)

    def as_dict(self):
        return {'name': self.name, 'kind': self.kind, 'x': r(self.x), 'y': r(self.y),
                'width': r(self.width), 'depth': r(self.depth)}


def r(value, digits=3):
    return round(value, digits)


def snap(value, step=0.05):
    return round(value / step) * step


def overlaps_start(x0, y0, x1, y1):
    sx0, sy0, sx1, sy1 = START_KEEP_CLEAR
    return x0 < sx1 and x1 > sx0 and y0 < sy1 and y1 > sy0


def wall_with_door(rng, name, vertical, at, start, end):
    """Wall along x=at (vertical) or y=at from start to end with one doorway."""
    door = snap(rng.uniform(*DOOR_WIDTH))
    door_start = snap(rng.uniform(start, end - door))
    pieces = []
    for index, (a, b) in enumerate(((start, door_start), (door_start + door, end))):
        if b - a < 0.4:  # no stubs: the doorway then runs to the end
            continue
        middle, length = (a + b) / 2, b - a
        if vertical:
            pieces.append(Box(f'{name}{"ab"[index]}', 'wall', at, middle, WALL_THICKNESS,
                              length))
        else:
            pieces.append(Box(f'{name}{"ab"[index]}', 'wall', middle, at, length,
                              WALL_THICKNESS))
    return pieces, (door_start, door_start + door)


def divide(rng, region, depth, walls, doors):
    """Recursive division of region (x0, y0, x1, y1) into rooms joined by doorways."""
    x0, y0, x1, y1 = region
    if depth == 0 or (walls and rng.random() < 0.25):
        return
    width, height = x1 - x0, y1 - y0
    choices = []
    if width >= 2 * ROOM_MIN:
        choices.append(True)
    if height >= 2 * ROOM_MIN:
        choices.append(False)
    rng.shuffle(choices)
    choices.sort(key=lambda vertical: -(width if vertical else height))
    for vertical in choices:
        for _ in range(20):
            low, high = (x0, x1) if vertical else (y0, y1)
            at = snap(rng.uniform(low + ROOM_MIN, high - ROOM_MIN))
            start, end = (y0, y1) if vertical else (x0, x1)
            # A wall must not end inside a doorway of the wall it meets.
            if any(door_at in (start, end) and lo - 0.15 <= at <= hi + 0.15
                   for door_vertical, door_at, lo, hi in doors if door_vertical != vertical):
                continue
            pieces, door = wall_with_door(rng, f'W{len(doors) + 1}', vertical, at, start, end)
            if any(overlaps_start(p.x - p.width / 2, p.y - p.depth / 2,
                                  p.x + p.width / 2, p.y + p.depth / 2) for p in pieces):
                continue
            walls.extend(pieces)
            doors.append((vertical, at) + door)
            halves = ([(x0, y0, at, y1), (at, y0, x1, y1)] if vertical
                      else [(x0, y0, x1, at), (x0, at, x1, y1)])
            for half in halves:
                divide(rng, half, depth - 1, walls, doors)
            return


def place_obstacles(rng, count, boxes):
    placed = []
    for index in range(count):
        for _ in range(200):
            width = snap(rng.uniform(*OBSTACLE_SIZE))
            depth = snap(rng.uniform(*OBSTACLE_SIZE))
            x = snap(rng.uniform(0.3 + width / 2, ARENA - 0.3 - width / 2))
            y = snap(rng.uniform(0.3 + depth / 2, ARENA - 0.3 - depth / 2))
            if overlaps_start(x - width / 2 - PASSAGE_M / 2, y - depth / 2 - PASSAGE_M / 2,
                              x + width / 2 + PASSAGE_M / 2, y + depth / 2 + PASSAGE_M / 2):
                continue
            if all(b.gap(x, y, width, depth) >= PASSAGE_M for b in boxes + placed):
                placed.append(Box(f'O{index + 1}', 'obstacle', x, y, width, depth))
                break
    return placed


def place_markers(rng, markers, boxes):
    """Stands either backed against a wall, facing away from it, or free-standing."""
    walls = [b for b in boxes if b.kind == 'wall']
    placed = []
    for name, marker_id, target_id, kind in markers:
        for _ in range(500):
            host = None
            if rng.random() < 0.5:
                host = rng.choice(walls)
                vertical = host.depth > host.width
                along = host.depth if vertical else host.width
                if along < 1.0:
                    continue
                side = rng.choice((-1, 1))
                # Half a metre from the piece's ends keeps the stand out of doorways.
                offset = rng.uniform(-along / 2 + 0.5, along / 2 - 0.5)
                standoff = WALL_THICKNESS / 2 + STAND_DEPTH + 0.01
                if vertical:
                    yaw = 0.0 if side > 0 else math.pi
                    x, y = host.x + side * standoff, host.y + offset
                else:
                    x, y, yaw = host.x + offset, host.y + side * standoff, side * math.pi / 2
            else:
                x, y = rng.uniform(0.6, ARENA - 0.6), rng.uniform(0.6, ARENA - 0.6)
                yaw = rng.uniform(-math.pi, math.pi)
            if not (0.25 < x < ARENA - 0.25 and 0.25 < y < ARENA - 0.25):
                continue
            if overlaps_start(x - 0.6, y - 0.6, x + 0.6, y + 0.6):
                continue
            if any(b.gap(x, y, 2 * STAND_RADIUS, 2 * STAND_RADIUS) < PASSAGE_M
                   for b in boxes if b is not host):
                continue
            if any(math.hypot(x - m['x'], y - m['y']) < MARKER_SPACING_M for m in placed):
                continue
            placed.append({'name': name, 'marker_id': marker_id, 'target_id': target_id,
                           'type': kind, 'x': r(x), 'y': r(y), 'z': r(BASE_HEIGHT / 2 + BOARD_Z),
                           'yaw': r(math.atan2(math.sin(yaw), math.cos(yaw)), 4),
                           'mounting': 'wall' if host else 'free'})
            break
        else:
            raise RuntimeError(f'no place for marker {name}')
    return placed


def stand_column(marker):
    """Column footprint behind the board: centre, yaw and extents (the LiDAR sees only it)."""
    model_yaw = marker['yaw'] + math.pi / 2
    behind = STAND_DEPTH / 2
    return (marker['x'] - math.sin(model_yaw) * behind,
            marker['y'] + math.cos(model_yaw) * behind, model_yaw)


def occupied(x, y, boxes, markers, margin=0.0):
    if any(b.contains(x, y, margin) for b in boxes):
        return True
    for marker in markers:
        cx, cy, yaw = stand_column(marker)
        dx, dy = x - cx, y - cy
        u = dx * math.cos(yaw) + dy * math.sin(yaw)
        v = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if abs(u) <= STAND_WIDTH / 2 + margin and abs(v) <= STAND_DEPTH / 2 + margin:
            return True
    return False


def blocks_handover(boxes, markers, spots):
    """True if geometry or a marker stand enters a handover pose's turning circle."""
    for sx, sy in spots:
        for b in boxes:
            dx = max(abs(sx - b.x) - b.width / 2, 0.0)
            dy = max(abs(sy - b.y) - b.depth / 2, 0.0)
            if math.hypot(dx, dy) < HANDOVER_CLEAR_M:
                return True
        if any(math.hypot(m['x'] - sx, m['y'] - sy) < HANDOVER_CLEAR_M + STAND_RADIUS
               for m in markers):
            return True
    return False


def reachable_cells(boxes, markers, starts):
    """Cells the inflated robot can reach from the start poses, and all free cells."""
    size = int(round(ARENA / GRID_M))
    centre = [(i + 0.5) * GRID_M for i in range(size)]
    free = {(i, j) for i in range(size) for j in range(size)
            if not occupied(centre[i], centre[j], boxes, markers, ROBOT_CLEARANCE_M)}
    queue = [(int(x / GRID_M), int(y / GRID_M)) for x, y in starts]
    seen = {cell for cell in queue if cell in free}
    queue = list(seen)
    while queue:
        i, j = queue.pop()
        for cell in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if cell in free and cell not in seen:
                seen.add(cell)
                queue.append(cell)
    return seen, free


def layout(seed, markers, obstacle_range, depth):
    rng = random.Random(seed)
    starts = load_starts()
    spots = load_handover_spots()
    for _ in range(100):
        boxes = [Box('south', 'wall', ARENA / 2, 0.0, ARENA + WALL_THICKNESS, WALL_THICKNESS),
                 Box('north', 'wall', ARENA / 2, ARENA, ARENA + WALL_THICKNESS, WALL_THICKNESS),
                 Box('west', 'wall', 0.0, ARENA / 2, WALL_THICKNESS, ARENA + WALL_THICKNESS),
                 Box('east', 'wall', ARENA, ARENA / 2, WALL_THICKNESS, ARENA + WALL_THICKNESS)]
        interior = []
        divide(rng, (0.0, 0.0, ARENA, ARENA), depth, interior, [])
        boxes += interior
        boxes += place_obstacles(rng, rng.randint(*obstacle_range), boxes)
        try:
            placed = place_markers(rng, markers, boxes)
        except RuntimeError:
            continue
        if blocks_handover(boxes, placed, spots):
            continue
        reach, free = reachable_cells(boxes, placed, [(s['x'], s['y']) for s in starts.values()])
        views = [(int((m['x'] + VIEW_DISTANCE_M * math.cos(m['yaw'])) / GRID_M),
                  int((m['y'] + VIEW_DISTANCE_M * math.sin(m['yaw'])) / GRID_M))
                 for m in placed]
        # Every room reachable (a few cells may be cut off in corners) and every marker seen.
        views += [(int(x / GRID_M), int(y / GRID_M)) for x, y in spots]
        if len(reach) >= 0.97 * len(free) and all(view in reach for view in views):
            return boxes, placed, starts
    raise RuntimeError(f'seed {seed}: no connected layout in 100 attempts')


def load_starts():
    """Start poses inside the arena; robots waiting outside it join only at the handover."""
    robots = yaml.safe_load((REPOSITORY / 'config/sim/robots_midterm.yaml').read_text())
    return {robot_id: pose for robot_id, pose in robots['robots'].items()
            if 0.0 < pose['x'] < ARENA and 0.0 < pose['y'] < ARENA}


def load_handover_spots():
    parameters = yaml.safe_load(
        (REPOSITORY / 'config/competition_parameters.yaml').read_text())
    poses = parameters['development']['localization']['start_poses']
    return [(pose['x'], pose['y']) for name, pose in sorted(poses.items())
            if name.startswith('handover_')]


def load_markers(distractors):
    parameters = yaml.safe_load(
        (REPOSITORY / 'config/competition_parameters.yaml').read_text())
    mapping = parameters['development']['aruco']['id_mapping']
    markers = [(entry['target_id'], int(marker_id), entry['target_id'], entry['type'])
               for marker_id, entry in sorted(mapping.items())]
    spare = [i for i in range(10, 50) if i not in mapping]
    markers += [(f'X{marker_id}', marker_id, None, 'unknown')
                for marker_id in spare[:distractors]]
    return markers


def sub(parent, tag, text=None, **attributes):
    item = ET.SubElement(parent, tag, attributes)
    if text is not None:
        item.text = str(text)
    return item


def box_geometry(link, name, width, depth, height, *, pose=None, collision=True, color=None):
    for kind in (('collision', 'visual') if collision else ('visual',)):
        element = sub(link, kind, name=name + '_' + kind)
        if pose:
            sub(element, 'pose', pose)
        geometry = sub(element, 'geometry')
        sub(sub(geometry, 'box'), 'size', f'{width:.4f} {depth:.4f} {height:.4f}')
        if kind == 'visual' and color:
            material = sub(element, 'material')
            sub(material, 'ambient', color)
            sub(material, 'diffuse', color)


def contact_sensor(link, name):
    sensor = sub(link, 'sensor', name='contact', type='contact')
    sub(sensor, 'always_on', 'true')
    sub(sensor, 'update_rate', 30)
    sub(sensor, 'topic', f'/rescue_contacts/{name}')
    contact = sub(sensor, 'contact')
    sub(contact, 'collision', 'body_collision')
    sub(contact, 'topic', f'/rescue_contacts/{name}')


def write_world(path, boxes, markers):
    import cv2
    root = ET.Element('sdf', version='1.9')
    # Name 'empty': the robot stack's world services and spawn interfaces expect it.
    world = sub(root, 'world', name='empty')
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
    sub(world, 'plugin', filename='edgeai_contacts', name='edgeai::Contacts')
    floor = sub(world, 'model', name='ground_plane')
    sub(floor, 'static', 'true')
    floor_link = sub(floor, 'link', name='link')
    for kind in ('collision', 'visual'):
        plane = sub(sub(sub(floor_link, kind, name=kind), 'geometry'), 'plane')
        sub(plane, 'normal', '0 0 1')
        sub(plane, 'size', '100 100')
    for item in boxes:
        model = sub(world, 'model', name=item.name)
        sub(model, 'static', 'true')
        sub(model, 'pose', f'{item.x:.4f} {item.y:.4f} {WALL_HEIGHT / 2} 0 0 0')
        link = sub(model, 'link', name='link')
        color = '0.35 0.35 0.35 1' if item.kind == 'wall' else '0.55 0.45 0.35 1'
        box_geometry(link, 'body', item.width, item.depth, WALL_HEIGHT, color=color)
        contact_sensor(link, item.name)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    tile = MARKER_SIDE / 6
    for marker in markers:
        # Model frame as the midterm S1 stand: the marker faces -y, the column stands
        # behind it at +y; the model yaw turns -y onto the marker normal (marker yaw).
        name = marker['name'] + '_stand'
        model = sub(world, 'model', name=name)
        sub(model, 'static', 'true')
        sub(model, 'pose', f'{marker["x"]:.4f} {marker["y"]:.4f} {BASE_HEIGHT / 2} 0 0 '
                           f'{marker["yaw"] + math.pi / 2:.5f}')
        link = sub(model, 'link', name='link')
        # The column reaches above the 0.155 m LiDAR plane so SLAM and the collision
        # monitor see the stand; it is set behind the board and does not hide the marker.
        box_geometry(link, 'body', STAND_WIDTH, STAND_DEPTH, STAND_HEIGHT,
                     pose=f'0 {STAND_DEPTH / 2} {STAND_HEIGHT / 2 - BASE_HEIGHT / 2} 0 0 0',
                     color='0.35 0.35 0.35 1')
        contact_sensor(link, name)
        box_geometry(link, 'board', BOARD_SIDE, BOARD_THICKNESS, BOARD_SIDE,
                     pose=f'0 0 {BOARD_Z} 0 0 0', color='1 1 1 1')
        pattern = cv2.aruco.generateImageMarker(dictionary, marker['marker_id'], 6)
        face = -(BOARD_THICKNESS / 2 + 0.0005)
        for row in range(6):
            for col in range(6):
                # Seen from the front (-y), screen right is +x.
                px, pz = (col - 2.5) * tile, BOARD_Z + (2.5 - row) * tile
                color = '0 0 0 1' if pattern[row, col] == 0 else '1 1 1 1'
                box_geometry(link, f'pixel_{row}_{col}', tile, 0.001, tile,
                             pose=f'{px:.5f} {face} {pz:.5f} 0 0 0', collision=False,
                             color=color)
    light = sub(world, 'light', name='sun', type='directional')
    sub(light, 'pose', '0 0 10 0 0 0')
    sub(light, 'diffuse', '0.8 0.8 0.8 1')
    sub(light, 'direction', '-0.3 0.2 -1')
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def write_truth_map(output, boxes, markers):
    resolution, origin, size = 0.02, -0.2, int(round((ARENA + 0.4) / 0.02))
    pixels = bytearray()
    for row in reversed(range(size)):
        y = origin + (row + 0.5) * resolution
        for col in range(size):
            x = origin + (col + 0.5) * resolution
            pixels.append(0 if occupied(x, y, boxes, markers) else 254)
    (output / 'truth_map.pgm').write_bytes(f'P5\n{size} {size}\n255\n'.encode() + pixels)
    (output / 'truth_map.yaml').write_text(yaml.safe_dump({
        'image': 'truth_map.pgm', 'mode': 'trinary', 'resolution': resolution,
        'origin': [origin, origin, 0.0], 'negate': 0,
        'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False))


def generate(output, seed, distractors=0, obstacles=(1, 3), depth=2):
    boxes, markers, starts = layout(seed, load_markers(distractors), obstacles, depth)
    output.mkdir(parents=True, exist_ok=True)
    write_world(output / 'arena.sdf', boxes, markers)
    write_truth_map(output, boxes, markers)
    truth = {
        'schema': 'edgeai-practice-arena/1',
        'seed': seed,
        'note': 'Ground truth for scoring only; never give it to the robots.',
        'arena': {'width_m': ARENA, 'height_m': ARENA,
                  'frame': 'arena, origin at the lower-left corner'},
        'start_poses': starts,
        'boxes': [b.as_dict() for b in boxes],
        'markers': markers,
    }
    (output / 'ground_truth.json').write_text(json.dumps(truth, indent=2) + '\n')
    return truth


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('output', type=Path, nargs='?',
                        help='default: runs/practice-arenas/seed-<seed>')
    parser.add_argument('--seed', type=int, default=None, help='default: random')
    parser.add_argument('--distractors', type=int, default=0,
                        help='extra markers with IDs outside the id_mapping')
    parser.add_argument('--obstacles', type=int, nargs=2, default=(1, 3),
                        metavar=('MIN', 'MAX'), help='number of free obstacles')
    parser.add_argument('--depth', type=int, default=2,
                        help='room division depth (0: one open room)')
    options = parser.parse_args()
    seed = options.seed if options.seed is not None else random.SystemRandom().randrange(10**6)
    output = options.output or Path('runs/practice-arenas') / f'seed-{seed}'
    truth = generate(output, seed, options.distractors, tuple(options.obstacles), options.depth)
    walls = sum(b['kind'] == 'wall' for b in truth['boxes']) - 4
    obstacles = sum(b['kind'] == 'obstacle' for b in truth['boxes'])
    print(f'seed {seed}: {walls} interior wall pieces, {obstacles} obstacles, markers '
          + ', '.join(f'{m["name"]}({m["marker_id"]}) at ({m["x"]}, {m["y"]})'
                      for m in truth['markers']))
    print(f'world: {output / "arena.sdf"}')


if __name__ == '__main__':
    main()
