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

"""Read immutable rescue inputs written by the Task 1 finalization and phase manager."""

from dataclasses import dataclass
import hashlib
from itertools import combinations
import json
import math
from pathlib import Path

from edgeai_common.config import resolve_parameters
import yaml


@dataclass(frozen=True)
class Rules:
    """One run's versioned rescue and dwell parameters."""

    geometry_version: str
    tolerance: float
    arrival: float
    normal_dwell: float
    cooperative_dwell: float

    @classmethod
    def load(cls, path, phase, config_dir, *, test_mode):
        mode = ('development', 'rehearsal', 'competition')[phase.run_mode]
        if test_mode and not path and mode == 'development':
            rescue = resolve_parameters(mode, config_dir).values['rescue']
        else:
            data = Path(path).read_bytes()
            if hashlib.sha256(data).hexdigest() != phase.parameters_sha256:
                raise ValueError('resolved parameter file does not match competition state')
            document = yaml.safe_load(data)
            if document['run_mode'] != mode:
                raise ValueError('resolved run mode does not match competition state')
            rescue = document['values']['rescue']
        tolerance = rescue['position_tolerance_m']
        # The optional engineering tolerance defaults to half the scoring tolerance.
        arrival = rescue.get('arrival_xy_tolerance_m')
        if arrival is None and tolerance is not None:
            arrival = tolerance / 2
        dwell = rescue['normal_dwell_s'], rescue['cooperative_dwell_s']
        geometry = rescue['geometry_version']
        if (not geometry
                or not all(isinstance(x, (int, float)) and not isinstance(x, bool)
                           and math.isfinite(x) and x > 0 for x in (tolerance, arrival, *dwell))
                or arrival >= tolerance):
            raise ValueError('invalid or missing resolved rescue geometry, tolerance or dwell')
        return cls(geometry, tolerance, arrival, *dwell)


def context(message):
    """Identity shared by competition state and a validated plan."""
    return message.run_id, message.map_id, message.map_revision, message.target_revision


def load_targets(path, identity, geometry_version, *, official):
    """Validate matching, versioned Task 1 output before building the rescue queue."""
    data = Path(path).read_bytes()
    doc = json.loads(data)
    frozen_identity = (doc['run_id'], doc['map']['map_id'], doc['map']['revision'], doc['revision'])
    if (doc['schema_version'] != 2 or not doc['frozen_at']
            or doc['frame_id'] != 'map' or doc['map']['frame_id'] != 'map'
            or frozen_identity != identity):
        raise ValueError('frozen targets do not match the active run/map/target revisions')
    geometry = doc['rescue_geometry']
    if geometry['version'] != geometry_version or (official and geometry['source'] != 'official'):
        raise ValueError('frozen target geometry does not match the resolved rescue rules')
    targets = {}
    for row in doc['targets']:
        target_id = row['target_id']
        if not target_id or target_id in targets:
            raise ValueError('empty or duplicate target ID in frozen registry')
        zones = sorted(row['rescue_zones'], key=lambda z: z['index'])
        count = {'normal': 1, 'cooperative': 2}.get(row['target_type'])
        if count is None or [z['index'] for z in zones] != list(range(count)):
            raise ValueError(f'{target_id}: invalid rescue zone indices')
        slots = [zone_pose(z['pose']) for z in zones]
        if any(a[:2] == b[:2] for a, b in combinations(slots, 2)):
            raise ValueError(f'{target_id}: duplicate rescue zone positions')
        targets[target_id] = {'zones': zones, 'slots': slots,
                              'eligible': row['eligible_for_basic_rescue'] is True}
    return targets, hashlib.sha256(data).hexdigest()


def zone_pose(pose):
    """Return the planar (x, y, yaw) for a frozen zone pose."""
    x, y, z = (pose['position'][key] for key in ('x', 'y', 'z'))
    q = [pose['orientation'][key] for key in ('x', 'y', 'z', 'w')]
    if (not all(math.isfinite(v) for v in (x, y, z, *q))
            or abs(z) > 1e-3 or abs(q[0]) > 1e-3 or abs(q[1]) > 1e-3
            or abs(sum(v*v for v in q) - 1) > 1e-3):
        raise ValueError('invalid planar rescue zone pose')
    return x, y, math.atan2(2 * q[2] * q[3], 1 - 2 * q[2]**2)
