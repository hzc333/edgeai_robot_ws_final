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

"""Independently audit the final ten seconds before each rescue completion event."""

import argparse
import hashlib
import json
import math
from pathlib import Path


def audit(directory):
    """Use raw Gazebo positions, without importing the coordinator's verifier or engine."""
    report = json.loads((directory / 'report.json').read_text())
    event_files = list((directory / 'evidence').glob('*/events.jsonl'))
    if len(event_files) != 1:
        raise ValueError('expected exactly one rescue evidence run')
    events = [json.loads(line) for line in event_files[0].read_text().splitlines()]
    lines = (directory / 'ground_truth.jsonl').read_text().splitlines()
    rows = [json.loads(line) for line in lines]
    completed = [e for e in events if e['type'] == 'COMPLETED']
    collision_events = [e for e in events if e['type'] == 'COLLISION']
    observer = report.get('collision_observer', {})
    physics_valid = (observer.get('state') == 'CLEAR' and not observer.get('ever_contact', True)
                     and observer.get('count') == 0
                     and all(observer.get('coverage', {}).get(r, 0) > 0
                             and r in observer.get('physics_alive', [])
                             for r in ('robot_1', 'robot_2')))
    injections = report.get('recovery_injections', [])
    recovered = [e for e in events if e['type'] == 'RECOVERED']
    recovery_valid = not injections or (
        {i['kind'] for i in injections} == {'freeze_during_s1', 'sigkill_restart_during_c1'}
        and len(recovered) >= 2
        and not any(e['type'] == 'COMPLETED' and e['dwell_elapsed_s'] < 10 for e in completed))
    rules = {'S1': ((1.0, 4.1), 0.5, 1), 'C1': ((4.1, 1.0), 0.7, 2)}
    checks = []
    for target_id, (center, radius, count) in rules.items():
        matches = [e for e in completed if e['target_id'] == target_id]
        if len(matches) != 1:
            checks.append({'target': target_id, 'passed': False,
                           'reason': 'missing or duplicate completion'})
            continue
        event = matches[0]
        end = event['sim_time']
        start = end - 10.0
        robots = event['robot_ids']
        window = [r for r in rows if start <= r['sim_time'] <= end]
        distances = {rid: [] for rid in robots}
        for row in window:
            for rid in robots:
                pose = row['poses'].get(rid)
                distances[rid].append(math.hypot(pose[1] - center[0], pose[2] - center[1])
                                      if pose else math.inf)
        gaps = [b['sim_time'] - a['sim_time'] for a, b in zip(window, window[1:])]
        coverage = bool(window and window[0]['sim_time'] - start <= 0.05
                        and end - window[-1]['sim_time'] <= 0.05
                        and gaps and max(gaps) <= 0.1)
        passed = (len(set(robots)) == count and coverage and event['dwell_elapsed_s'] >= 10
                  and all(values and max(values) <= radius for values in distances.values()))
        checks.append({'target': target_id, 'passed': passed, 'window_sim_s': [start, end],
                       'samples': len(window), 'maximum_sample_gap_s': max(gaps) if gaps else None,
                       'radius_m': radius,
                       'maximum_distances_m': {r: max(v) if v else None
                                               for r, v in distances.items()},
                       'recorded_dwell_s': event['dwell_elapsed_s']})
    physics_replay = bool(observer.get('observer_instance')
                          and observer.get('last_sequence') is not None)
    result = {
        'scope': 'Independent raw-Gazebo audit at each completion; sampled collision evidence',
        'run_id': report['run_id'], 'checks': checks,
        'footprint_overlap_samples': len(report['footprint_envelope_overlaps']),
        'physics_contact_messages': len(report['physics_contacts']),
        'physics_sensor_delivery': 'verified' if physics_valid else 'not established',
        'collision_observer': observer, 'collision_events': len(collision_events),
        'physics_history_replay': physics_replay,
        'recovery_injections': injections, 'recovery_confirmed': recovery_valid,
        'recovery_events': len(recovered),
        'freeze_safe_confirmed': report['freeze_safe_confirmed'],
        'part2_wall_elapsed_s': report['part2_wall_elapsed_s'],
        'raw_files_sha256': {
            str(p.relative_to(directory)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (event_files[0], directory / 'ground_truth.jsonl',
                      directory / 'source_manifest.json') if p.exists()},
        'passed': (all(c['passed'] for c in checks) and len(completed) == 2
                   and not report['footprint_envelope_overlaps']
                   and not report['physics_contacts'] and report['freeze_safe_confirmed']
                   and physics_valid and not collision_events and recovery_valid
                   and report['rescue_status']['status'] == 'DONE' and not report['error']),
    }
    (directory / 'audited_report.json').write_text(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    result = audit(parser.parse_args().directory)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
