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
Midterm check result verification (task description section 6) for one run directory.

Prints the timing, the saved Task 1 map, the ArUco detection evidence of S1 and C1,
the S1 hold and the simultaneous C1 hold (recomputed from P4's distance history, in
simulation time) and the collision status. Writes two figures:

  map.png        saved map with the rescue radii, official and detected target positions
  distances.png  robot-to-target distance history with the radii and the hold windows

    python tools/midterm_report.py runs/midterm-rehearsal/<run_id> [--out DIR] [--open]

The figures go to <run_dir>.verification/ next to the run directory (the sealed run
directory itself stays unchanged). Exit status 0 when every check passes.
"""

import argparse
from datetime import datetime
import glob
import json
import math
from pathlib import Path
import subprocess
import sys

START_AREA = (0.0, 1.4, 0.0, 1.0)       # x min, x max, y min, y max [m]


def read_jsonl(path):
    with open(path) as stream:
        return [json.loads(line) for line in stream if line.strip()]


def wall(event):
    return datetime.fromisoformat(event['wall_time'].replace('Z', '+00:00')).timestamp()


def load_map(task1):
    import numpy as np
    import yaml
    meta = yaml.safe_load((task1 / 'map.yaml').read_text())
    with open(task1 / meta['image'], 'rb') as stream:
        if stream.readline().strip() != b'P5':
            raise ValueError('map image is not a binary PGM')
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        width, height = map(int, line.split())
        stream.readline()
        image = np.frombuffer(stream.read(), np.uint8).reshape(height, width)
    return image, meta


def holds(rows, target_id):
    """Longest stretch in which every assigned robot stays within the radius."""
    rows = [r for r in rows if r.get('target_id') == target_id and r.get('assignment')]
    best, start, last = None, None, None
    for row in rows:
        inside = all(row['distances_m'].get(robot, math.inf) <= row['radius_m']
                     for robot in row['assignment'])
        if inside:
            start = start or row
            last = row
            if best is None or last['sim_time'] - start['sim_time'] > best[2]:
                best = (start, last, last['sim_time'] - start['sim_time'])
        else:
            start = None
    if best is None:
        return None
    return {'robots': best[0]['assignment'], 'radius_m': best[0]['radius_m'],
            'required_s': best[0]['required_dwell_s'], 'center': best[0]['center'],
            'from_s': best[0]['sim_time'], 'to_s': best[1]['sim_time'],
            'duration_s': best[2]}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--out', type=Path, help='figure directory')
    parser.add_argument('--open', action='store_true', help='open the figures afterwards')
    options = parser.parse_args()
    run = options.run_dir.resolve()
    run_id = run.name
    out = (options.out or run.with_name(run_id + '.verification')).resolve()
    checks = {}

    def section(title):
        print('\n== ' + title)

    print(f'Midterm verification: {run_id}')

    # Timing (13 min shared timer: from the start of exploration to the end of the rescue).
    section('Timing')
    events = read_jsonl(run / 'events.jsonl')
    phases = {e['phase']: e for e in events if e['category'] == 'phase'}
    for name in ('task1', 'handover', 'task2a', 'frozen', 'finished'):
        if name in phases:
            e = phases[name]
            print(f'  {name:9s} sim {e["ros_time_ns"] / 1e9:7.1f} s   '
                  f'wall {wall(e) - wall(phases["task1"]):6.1f} s after the start')
    if 'task1' in phases and 'frozen' in phases:
        total = wall(phases['frozen']) - wall(phases['task1'])
        checks['finished within 13 min'] = total <= 780
        print(f'  task time (exploration start -> rescue end): {total:.1f} s wall '
              f'({total / 60:.1f} of 13 min)')

    # Part 1 map.
    section('Part 1 map (task1/)')
    image, meta = load_map(run / 'task1')
    quality = json.loads((run / 'task1' / 'map_quality.json').read_text())
    height, width = image.shape
    resolution = meta['resolution']
    print(f'  {run / "task1" / "map.yaml"}')
    print(f'  {width} x {height} cells of {resolution:.2f} m, origin '
          f'({meta["origin"][0]:.3f}, {meta["origin"][1]:.3f}); known area '
          f'{quality["known_area_m2"]:.1f} m2, free {quality["free_area_m2"]:.1f} m2')
    checks['map saved'] = True

    # Detection evidence.
    section('ArUco detection evidence (camera frame, simulation time)')
    detections_dir = run / 'detections' / run_id
    # Part 1 evidence only: detections up to the end of exploration (Task 2 also detects).
    end = phases['handover']['ros_time_ns'] / 1e9 if 'handover' in phases else math.inf
    records = [r for r in read_jsonl(detections_dir / 'detections.jsonl')
               if r['sim_time'] <= end]
    targets = {t['target_id']: t for t in
               json.loads((run / 'task1' / 'targets.json').read_text())['targets']}
    for target_id in ('S1', 'C1'):
        own = [r for r in records if r['target_id'] == target_id]
        if not own:
            print(f'  {target_id}: NOT DETECTED')
            checks[target_id + ' detected'] = False
            continue
        best = max(own, key=lambda r: r['confidence'])
        x, y, z = best['position_m']
        print(f'  {target_id}: marker ID {best["marker_id"]}, {best["target_type"]}; '
              f'{len(own)} records, first at sim {own[0]["sim_time"]:.1f} s')
        print(f'      best: sim {best["sim_time"]:.1f} s by {best["robot_id"]}, '
              f'position ({x:+.3f}, {y:+.3f}, {z:+.3f}) m in {best["frame_id"]}, '
              f'range {best["range_m"]:.2f} m, confidence {best["confidence"]:.2f}')
        if target_id in targets:
            p = targets[target_id]['marker_pose']['position']
            print(f'      fused map position ({p["x"]:.3f}, {p["y"]:.3f}) m')
        images = sorted(i for i in detections_dir.glob(target_id + '_*.png')
                        if float(i.stem.rsplit('_', 1)[1]) <= end)
        if images:
            print(f'      overlay images: {len(images)}, e.g. {images[0]}')
        checks[target_id + ' detected'] = True

    # Rescue holds.
    section('Part 2 rescue holds (P4 distance history, simulation time)')
    p4 = sorted(glob.glob(str(run / 'p4' / '*' / 'distances.jsonl')))
    rows = read_jsonl(p4[0]) if p4 else []
    snapshot = json.loads(Path(p4[0]).with_name('snapshot.json').read_text()) if p4 else {}
    completed = set(snapshot.get('completed_targets', []))
    dwell = {}
    for e in events:
        if e['category'] == 'rescue' and e['type'] in ('event_dwell_started',
                                                       'event_completed'):
            dwell.setdefault(e['payload']['target_id'], {})[e['type']] = e['ros_time_ns'] / 1e9
    results = {}
    for target_id in ('S1', 'C1'):
        hold = results[target_id] = holds(rows, target_id)
        if hold is None:
            print(f'  {target_id}: no hold recorded')
            checks[target_id + ' rescued'] = False
            continue
        d = dwell.get(target_id, {})
        window = [r for r in rows if r.get('target_id') == target_id
                  and d.get('event_dwell_started', math.inf) <= r['sim_time']
                  <= d.get('event_completed', -math.inf)]
        distances = ', '.join(
            f'{robot} {max(w["distances_m"][robot] for w in window):.3f} m'
            for robot in hold['robots']) if window else ''
        print(f'  {target_id}: {" + ".join(hold["robots"])} within {hold["radius_m"]} m of '
              f'({hold["center"][0]}, {hold["center"][1]}) from sim {hold["from_s"]:.1f} '
              f'to {hold["to_s"]:.1f} s = {hold["duration_s"]:.1f} s continuous '
              f'(required {hold["required_s"]:.0f} s)')
        if d:
            print(f'      P4 hold: started {d.get("event_dwell_started", math.nan):.1f} s, '
                  f'completed {d.get("event_completed", math.nan):.1f} s; '
                  f'largest distance during the hold: {distances}')
        checks[target_id + ' rescued'] = (target_id in completed
                                          and hold['duration_s'] >= hold['required_s'])
        flag = 'COMPLETED' if target_id in completed else 'not completed'
        print(f'      completion flag: {flag}')

    collision = snapshot.get('collision_history', {})
    section('Collisions (Gazebo physics contacts, all robots)')
    print(f'  count {collision.get("count")}, first {collision.get("first")}')
    checks['no collisions'] = collision.get('count') == 0

    out.mkdir(parents=True, exist_ok=True)
    figures = plot(out, image, meta, targets, results, rows, dwell)
    section('Figures')
    for figure in figures:
        print('  ' + str(figure))

    section('Summary')
    for name, ok in checks.items():
        print(f'  [{"PASS" if ok else "FAIL"}] {name}')
    if options.open:
        for figure in figures:
            subprocess.Popen(['xdg-open', str(figure)], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
    return 0 if checks and all(checks.values()) else 1


def plot(out, image, meta, targets, results, rows, dwell):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle

    height, width = image.shape
    x0, y0 = meta['origin'][:2]
    extent = (x0, x0 + width * meta['resolution'], y0, y0 + height * meta['resolution'])
    official = {'S1': ((1.0, 4.1), 0.5, 'tab:red'), 'C1': ((4.1, 1.0), 0.7, 'tab:blue')}

    figure, axis = plt.subplots(figsize=(7, 7))
    axis.imshow(image, cmap='gray', extent=extent, origin='upper', vmin=0, vmax=255)
    axis.add_patch(Rectangle((START_AREA[0], START_AREA[2]), START_AREA[1] - START_AREA[0],
                             START_AREA[3] - START_AREA[2], fill=False, ls='--',
                             color='tab:green', label='start area'))
    for target_id, (center, radius, color) in official.items():
        axis.add_patch(Circle(center, radius, fill=False, ls='--', color=color))
        axis.plot(*center, '+', color=color, ms=12, mew=2,
                  label=f'{target_id} official ({center[0]}, {center[1]}), r {radius} m')
        if target_id in targets:
            p = targets[target_id]['marker_pose']['position']
            axis.plot(p['x'], p['y'], 'x', color=color, ms=9, mew=2,
                      label=f'{target_id} detected ({p["x"]:.2f}, {p["y"]:.2f})')
    axis.set(xlim=(-0.2, 5.2), ylim=(-0.2, 5.2), xlabel='x [m]', ylabel='y [m]',
             title='Part 1 saved map')
    axis.set_aspect('equal')
    axis.legend(loc='upper right', fontsize=7)
    figure.tight_layout()
    map_png = out / 'map.png'
    figure.savefig(map_png, dpi=130)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(9, 6.5))
    for axis, target_id in zip(axes, ('S1', 'C1')):
        own = [r for r in rows if r.get('target_id') == target_id and r.get('assignment')]
        robots = sorted({robot for r in own for robot in r['assignment']})
        for robot in robots:
            points = [(r['sim_time'], r['distances_m'][robot]) for r in own
                      if robot in r['distances_m']]
            if points:
                axis.plot(*zip(*points), label=robot)
        if own:
            axis.axhline(own[0]['radius_m'], color='k', ls='--',
                         label=f'radius {own[0]["radius_m"]} m')
        hold = results.get(target_id)
        if hold:
            axis.axvspan(hold['from_s'], hold['to_s'], color='tab:green', alpha=0.2,
                         label=f'all within radius {hold["duration_s"]:.1f} s')
        d = dwell.get(target_id, {})
        if 'event_dwell_started' in d and 'event_completed' in d:
            axis.axvspan(d['event_dwell_started'], d['event_completed'], color='tab:orange',
                         alpha=0.25, label='P4 10 s hold')
        axis.set(ylabel='distance [m]', title=f'{target_id}: robot to target centre',
                 ylim=(0, 3))
        axis.legend(loc='upper right', fontsize=7)
    axes[-1].set_xlabel('simulation time [s]')
    figure.tight_layout()
    distances_png = out / 'distances.png'
    figure.savefig(distances_png, dpi=130)
    plt.close(figure)
    return [map_png, distances_png]


if __name__ == '__main__':
    sys.exit(main())
