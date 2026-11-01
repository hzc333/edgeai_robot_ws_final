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

"""Versioned Task 1 artifact transaction with explicit crash recovery."""

import copy
import math
import os
import shutil

import yaml

from .storage import atomic_bytes, atomic_json, sha256, sync_directory, utc_now


def encode_map(grid):
    """Encode ROS bottom-up occupancy as top-down trinary PGM for Nav2."""
    pixels = bytearray()
    for row in range(grid.height - 1, -1, -1):
        for col in range(grid.width):
            value = grid.data[row * grid.width + col]
            pixels.append(205 if value < 0 or 25 < value < 65 else (254 if value <= 25 else 0))
    image = f'P5\n{grid.width} {grid.height}\n255\n'.encode() + pixels
    document = {'image': 'map.pgm', 'mode': 'trinary', 'resolution': grid.resolution,
                'origin': [grid.origin_x, grid.origin_y, grid.origin_yaw], 'negate': 0,
                'occupied_thresh': 0.65, 'free_thresh': 0.25}
    return image, yaml.safe_dump(document, sort_keys=True).encode()


def validate_targets(document):
    """Additional geometric invariants not expressible by the JSON Schema."""
    ids, markers = set(), set()
    geometry_known = document['rescue_geometry']['version'] is not None
    for target in document['targets']:
        if target['target_id'] in ids or target['marker_id'] in markers:
            raise ValueError('duplicate target or marker ID')
        ids.add(target['target_id'])
        markers.add(target['marker_id'])
        zones = target['rescue_zones']
        expected = (1 if target['target_type'] == 'normal' else 2) if geometry_known else 0
        if len(zones) != expected or [z['index'] for z in zones] != list(range(expected)):
            raise ValueError('rescue geometry mismatch')
        for pose in [target['marker_pose']] + [z['pose'] for z in zones]:
            q = pose['orientation']
            if abs(sum(v * v for v in q.values()) - 1.0) > 1e-4:
                raise ValueError('non-unit quaternion')
        covariance = target['covariance']
        if any(covariance[i * 6 + i] < 0 for i in range(6)):
            raise ValueError('negative covariance variance')
        if any(abs(covariance[i * 6 + j] - covariance[j * 6 + i]) > 1e-6
               for i in range(6) for j in range(6)):
            raise ValueError('asymmetric covariance')
        lower = [[0.0] * 6 for _ in range(6)]
        for i in range(6):
            for j in range(i + 1):
                residual = covariance[i * 6 + j] - sum(
                    lower[i][k] * lower[j][k] for k in range(j))
                if i == j:
                    if residual < -1e-8:
                        raise ValueError('covariance is not positive semidefinite')
                    lower[i][j] = math.sqrt(max(0, residual))
                elif lower[j][j] > 1e-10:
                    lower[i][j] = residual / lower[j][j]
                elif abs(residual) > 1e-8:
                    raise ValueError('covariance is not positive semidefinite')
        q = target['marker_pose']['orientation']
        yaw = math.atan2(2 * (q['w'] * q['z'] + q['x'] * q['y']),
                         1 - 2 * (q['y'] ** 2 + q['z'] ** 2))
        error = (yaw - target['marker_yaw_rad'] + math.pi) % (2 * math.pi) - math.pi
        if abs(error) > 1e-4:
            raise ValueError('marker yaw disagrees with quaternion')
        if not target['eligible_for_basic_rescue']:
            raise ValueError('frozen target is not Task 1 eligible')


class Task1Transaction:
    """Commit immutable files before atomically publishing the root manifest.

    A manifest intent lives inside the atomically renamed task1 directory. If
    the process dies after rename but before root publication, recover() can
    finish publication after validating every hash. Success is returned only
    after the root manifest is durable. Caller holds the RunStore lease.
    """

    def __init__(self, store, schemas):
        self.store, self.schemas = store, schemas

    def verify(self, manifest):
        self.schemas.check('manifest', manifest)
        if manifest['run_id'] != self.store.run_id:
            raise ValueError('manifest run mismatch')
        seen = set()
        for entry in manifest['files']:
            if entry['path'] in seen:
                raise ValueError('duplicate manifest path')
            seen.add(entry['path'])
            path = self.store.path(entry['path'])
            if path.stat().st_size != entry['size_bytes'] or sha256(path) != entry['sha256']:
                raise ValueError('frozen artifact changed: ' + entry['path'])
        return manifest

    def recover(self):
        current = self.store.read('manifest.json')
        intent = self.store.read('task1/.manifest-intent.json')
        if intent is None:
            if current and current['task1'] is not None:
                raise ValueError('frozen manifest has no transaction receipt')
            return current
        self.verify(intent)
        targets = self.store.read('task1/targets.json')
        self.schemas.check('target_list', targets)
        validate_targets(targets)
        if current and current['manifest_revision'] > intent['manifest_revision']:
            return self.verify(current)
        if current != intent:
            if current and current['task1'] is not None:
                raise ValueError('conflicting frozen manifest')
            self.store.write('manifest.json', intent)
        return intent

    def commit(self, grid, targets, base_manifest, map_id, map_revision,
               expected_target_revision, end_reason):
        current = self.recover()
        if current and current['task1'] is not None:
            frozen = current['task1']
            if (frozen['map_id'], frozen['map_revision'], frozen['target_revision']) != (
                    map_id, map_revision, expected_target_revision):
                raise ValueError('different snapshot already frozen')
            return self.verify(current)
        if grid is None or map_revision < 1:
            raise ValueError('map snapshot missing or unversioned')
        if (targets['run_id'] != self.store.run_id
                or targets['revision'] != expected_target_revision
                or targets['map']['map_id'] != map_id
                or targets['map']['revision'] != map_revision):
            raise ValueError('target/map snapshot identity mismatch')
        manifest = copy.deepcopy(base_manifest)
        self.schemas.check('manifest', manifest)
        if manifest['run_id'] != self.store.run_id or manifest['task1'] is not None:
            raise ValueError('invalid base manifest')
        # Prior immutable files must remain intact across every manifest revision.
        self.verify(manifest)
        staging = self.store.path('.task1-pending')
        if staging.exists():
            shutil.rmtree(staging)  # only this transaction's uncommitted staging area
        staging.mkdir()
        image, metadata = encode_map(grid)
        atomic_bytes(staging / 'map.pgm', image)
        atomic_bytes(staging / 'map.yaml', metadata)
        frozen_at = utc_now()
        target_doc = copy.deepcopy(targets)
        target_doc['frozen_at'] = frozen_at
        target_doc['map'] = {
            'map_id': map_id, 'revision': map_revision, 'frame_id': 'map',
            'yaml': 'task1/map.yaml', 'yaml_sha256': sha256(staging / 'map.yaml'),
            'image': 'task1/map.pgm', 'image_sha256': sha256(staging / 'map.pgm'),
            'resolution_m': grid.resolution,
            'origin': {'x': grid.origin_x, 'y': grid.origin_y, 'yaw': grid.origin_yaw},
            'width_cells': grid.width, 'height_cells': grid.height}
        self.schemas.check('target_list', target_doc)
        validate_targets(target_doc)
        atomic_json(staging / 'targets.json', target_doc)
        atomic_json(staging / 'map_metadata.json', target_doc['map'])
        metrics = grid.metrics()
        metrics['coverage_fraction'] = None  # no authoritative arena area supplied
        metrics['coverage_note'] = 'known area is measured; arena coverage is unavailable'
        atomic_json(staging / 'map_quality.json', metrics)
        manifest['manifest_revision'] += 1
        manifest['updated_at'] = frozen_at
        manifest['task1'] = {'frozen_at': frozen_at, 'map_id': map_id, 'map_revision': map_revision,
                             'target_revision': expected_target_revision, 'end_reason': end_reason}
        for name, role in [('map.pgm', 'map_image'), ('map.yaml', 'map_yaml'),
                           ('targets.json', 'target_list'), ('map_metadata.json', 'other'),
                           ('map_quality.json', 'other')]:
            path = staging / name
            manifest['files'].append({'path': 'task1/' + name, 'role': role, 'stage': 'task1',
                                      'sha256': sha256(path), 'size_bytes': path.stat().st_size})
        self.schemas.check('manifest', manifest)
        atomic_json(staging / '.manifest-intent.json', manifest)
        sync_directory(staging)
        destination = self.store.path('task1')
        if destination.exists():
            raise ValueError('task1 directory already exists without valid receipt')
        os.rename(staging, destination)
        sync_directory(self.store.directory)
        self.store.write('manifest.json', manifest)
        return self.verify(manifest)
