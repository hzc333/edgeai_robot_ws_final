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
Score a run's Task 1 output (task1/targets.json, task1/map.*) against an arena's ground truth.

The rulebook (5.5, 8) scores the number of correctly identified unique targets, their
position error and the exploration and mapping performance; its thresholds, map-alignment
method and false-detection handling are TBD. This tool reports the raw measures:

  targets   per ground-truth target: found, position error, yaw error; entries that are
            not ground-truth targets or repeat one (false detections)
  map       coverage: share of the arena's free floor the saved map knows as free;
            accuracy: share of the map's occupied cells within --wall-tolerance of true
            geometry. The map is assumed to be in the arena frame (the midterm scenario
            anchors SLAM at robot_1's start pose), so no alignment is done.

    python tools/score_task1.py runs/practice/<run_id> runs/practice-arenas/seed-7

Writes <run_dir>.task1_score.json next to the run directory; exit status 0 when every
ground-truth target was found within --max-error and nothing false was reported.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml


def load_pgm(yaml_path):
    meta = yaml.safe_load(yaml_path.read_text())
    with open(yaml_path.parent / Path(meta['image']).name, 'rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError(f'{yaml_path}: map image is not a binary PGM')
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        width, height = map(int, line.split())
        stream.readline()
        image = np.frombuffer(stream.read(), np.uint8).reshape(height, width)
    # Row 0 of the image is the top (largest y); flip so that [row, col] = [y, x].
    return image[::-1], meta


def cell_centres(image, meta):
    rows, cols = np.indices(image.shape)
    resolution = meta['resolution']
    return (meta['origin'][0] + (cols + 0.5) * resolution,
            meta['origin'][1] + (rows + 0.5) * resolution)


def sample(image, meta, x, y):
    """Values of a map at points (x, y); 205 (unknown) outside it."""
    resolution = meta['resolution']
    cols = np.floor((x - meta['origin'][0]) / resolution).astype(int)
    rows = np.floor((y - meta['origin'][1]) / resolution).astype(int)
    inside = (rows >= 0) & (rows < image.shape[0]) & (cols >= 0) & (cols < image.shape[1])
    values = np.full(x.shape, 205, np.uint8)
    values[inside] = image[rows[inside], cols[inside]]
    return values


def score_targets(targets, truth, max_error):
    expected = {m['target_id']: m for m in truth['markers'] if m['target_id']}
    rows, false, seen = {}, [], set()
    for target in targets:
        position = target['marker_pose']['position']
        entry = {'target_id': target['target_id'], 'marker_id': target.get('marker_id'),
                 'xy': [round(position['x'], 3), round(position['y'], 3)]}
        marker = expected.get(target['target_id'])
        if marker is None or target['target_id'] in seen:
            entry['reason'] = 'repeated target' if marker else 'not a ground-truth target'
            false.append(entry)
            continue
        seen.add(target['target_id'])
        error = math.hypot(position['x'] - marker['x'], position['y'] - marker['y'])
        yaw = target.get('marker_yaw_rad')
        rows[target['target_id']] = {
            **entry, 'found': True, 'error_m': round(error, 3),
            'yaw_error_deg': (round(math.degrees(abs(math.atan2(
                math.sin(yaw - marker['yaw']), math.cos(yaw - marker['yaw'])))), 1)
                if yaw is not None else None),
            'within_max_error': error <= max_error}
    for target_id, marker in expected.items():
        rows.setdefault(target_id, {'target_id': target_id, 'marker_id': marker['marker_id'],
                                    'found': False})
    return rows, false


def score_map(run_dir, arena, wall_tolerance):
    image, meta = load_pgm(run_dir / 'task1' / 'map.yaml')
    truth, truth_meta = load_pgm(arena / 'truth_map.yaml')
    # Free floor: true free cells inside the arena walls, away from the geometry by the
    # tolerance (a map cannot be blamed for half a cell at a wall).
    tx, ty = cell_centres(truth, truth_meta)
    margin = int(math.ceil(wall_tolerance / truth_meta['resolution']))
    occupied = truth < 128
    near = occupied.copy()
    for dr in range(-margin, margin + 1):
        for dc in range(-margin, margin + 1):
            near |= np.roll(np.roll(occupied, dr, 0), dc, 1)
    floor = ~near & (tx > 0) & (tx < 5.0) & (ty > 0) & (ty < 5.0)
    known_free = sample(image, meta, tx[floor], ty[floor]) > 250
    coverage = float(known_free.mean()) if floor.any() else 0.0
    # Accuracy: each occupied map cell must lie near true geometry.
    mx, my = cell_centres(image, meta)
    walls = image < 50
    near_truth = sample(np.where(near, 0, 254).astype(np.uint8), truth_meta,
                        mx[walls], my[walls]) == 0
    accuracy = float(near_truth.mean()) if walls.any() else None
    return {'coverage_fraction': round(coverage, 3),
            'occupied_accuracy_fraction': None if accuracy is None else round(accuracy, 3),
            'free_floor_m2': round(float(floor.sum()) * truth_meta['resolution'] ** 2, 2),
            'wall_tolerance_m': wall_tolerance}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('arena', type=Path, help='arena directory with ground_truth.json')
    parser.add_argument('--max-error', type=float, default=0.3,
                        help='position error that still counts as correct [m]')
    parser.add_argument('--wall-tolerance', type=float, default=0.1,
                        help='distance to true geometry an occupied map cell may have [m]')
    options = parser.parse_args()
    run_dir, arena = options.run_dir.resolve(), options.arena.resolve()
    truth = json.loads((arena / 'ground_truth.json').read_text())
    targets = json.loads((run_dir / 'task1' / 'targets.json').read_text())['targets']
    rows, false = score_targets(targets, truth, options.max_error)
    result = {'run': run_dir.name, 'arena': str(arena), 'seed': truth.get('seed'),
              'max_error_m': options.max_error, 'targets': rows, 'false_detections': false,
              'map': score_map(run_dir, arena, options.wall_tolerance)}
    correct = sum(row.get('within_max_error', False) for row in rows.values())
    result['correct_targets'] = correct
    result['passed'] = correct == len(rows) and not false
    out = run_dir.with_name(run_dir.name + '.task1_score.json')
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(f'Task 1 score, arena seed {truth.get("seed")}:')
    for target_id, row in sorted(rows.items()):
        if row['found']:
            print(f'  {target_id}: error {row["error_m"]} m, yaw error {row["yaw_error_deg"]} deg'
                  + ('' if row['within_max_error'] else f'  (over {options.max_error} m)'))
        else:
            print(f'  {target_id}: not found')
    for entry in false:
        print(f'  false: {entry["target_id"]} at {entry["xy"]} ({entry["reason"]})')
    print(f'  correct {correct}/{len(rows)}; map coverage {result["map"]["coverage_fraction"]}, '
          f'occupied accuracy {result["map"]["occupied_accuracy_fraction"]}')
    print(f'  {"PASS" if result["passed"] else "FAIL"}; details in {out}')
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
