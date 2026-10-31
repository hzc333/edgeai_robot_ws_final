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

"""ROS-independent Task 1 snapshot validation and atomic targets.json export."""

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import tempfile

from edgeai_robot_interfaces.msg import CompetitionState

import jsonschema

from .rescue_geometry import (
    normalize_angle,
    rescue_zone_poses,
    validate_development_geometry,
)


class SnapshotConflictError(RuntimeError):
    """Raised when a run's frozen target snapshot is requested with new versions."""


@dataclass(frozen=True)
class FrozenTargetSnapshot:
    """Immutable serialized target-registry snapshot for one Task 1 finalize request."""

    run_id: str
    map_id: str
    map_revision: int
    target_revision: int
    frozen_at: str
    targets_json: str

    @property
    def targets(self):
        """Return a fresh decoded copy of the captured targets."""
        return json.loads(self.targets_json)


class Task1SnapshotFreezer:
    """Capture a revision-matched TargetArray once per run and make retries idempotent."""

    def __init__(self):
        """Start with no frozen run snapshots."""
        self._snapshots = {}

    def freeze(self, target_array, competition_state, *, run_id, map_id,
               expected_map_revision, expected_target_revision, frozen_at):
        """Capture one exact run/map/target revision or reject inconsistent inputs."""
        _validate_target_metadata(target_array, competition_state)
        key = (
            run_id, map_id, int(expected_map_revision), int(expected_target_revision))
        previous = self._snapshots.get(run_id)
        if previous is not None:
            previous_key = (
                previous.run_id, previous.map_id,
                previous.map_revision, previous.target_revision)
            if previous_key != key:
                raise SnapshotConflictError(
                    'Task 1 snapshot for this run is already frozen at another revision')
            return previous

        if (not run_id or not map_id or competition_state.run_id != run_id
                or competition_state.map_id != map_id
                or competition_state.map_revision != expected_map_revision
                or competition_state.target_revision != expected_target_revision
                or competition_state.phase != CompetitionState.PHASE_TASK1
                or competition_state.frozen or competition_state.estop_active
                or competition_state.reset_in_progress):
            raise ValueError('CompetitionState does not match the Task 1 finalize request')
        if (target_array.run_id != run_id or target_array.map_id != map_id
                or target_array.map_revision != expected_map_revision
                or target_array.revision != expected_target_revision
                or target_array.header.frame_id != 'map'):
            raise ValueError('TargetArray does not match the requested frozen revisions')
        if not _valid_rfc3339(frozen_at):
            raise ValueError('frozen_at must be an RFC 3339 date-time with timezone')

        targets = [_target_to_dict(target) for target in target_array.targets]
        targets.sort(key=lambda item: item['target_id'])
        target_ids = [item['target_id'] for item in targets]
        if len(target_ids) != len(set(target_ids)):
            raise ValueError('TargetArray contains duplicate target IDs')
        snapshot = FrozenTargetSnapshot(
            run_id=run_id,
            map_id=map_id,
            map_revision=expected_map_revision,
            target_revision=expected_target_revision,
            frozen_at=frozen_at,
            targets_json=json.dumps(targets, sort_keys=True, separators=(',', ':')),
        )
        self._snapshots[run_id] = snapshot
        return snapshot


def _validate_target_metadata(target_array, competition_state):
    for target in target_array.targets:
        if (target.header.frame_id != target_array.header.frame_id
                or target.header.frame_id != 'map'
                or target.run_id != target_array.run_id
                or target.run_id != competition_state.run_id
                or target.map_id != target_array.map_id
                or target.map_id != competition_state.map_id
                or target.map_revision != target_array.map_revision
                or target.map_revision != competition_state.map_revision):
            raise ValueError(
                f'Target {target.target_id!r} metadata does not match its snapshot')


def _valid_rfc3339(value):
    if (not isinstance(value, str)
            or re.fullmatch(
                r'\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?(?:Z|\+00:00)',
                value) is None):
        return False
    try:
        from datetime import datetime
        datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError:
        return False
    return True


def _target_to_dict(target):
    pose = target.marker_pose.pose
    return {
        'target_id': target.target_id,
        'marker_id': target.marker_id,
        'target_type': {
            target.TYPE_NORMAL: 'normal',
            target.TYPE_COOPERATIVE: 'cooperative',
        }[target.target_type],
        'marker_pose': {
            'position': {
                'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
            },
            'orientation': {
                'x': pose.orientation.x, 'y': pose.orientation.y,
                'z': pose.orientation.z, 'w': pose.orientation.w,
            },
        },
        'covariance': list(target.marker_pose.covariance),
        'rescue_zones': [
            {
                'index': index,
                'pose': {
                    'position': {
                        'x': rescue.pose.position.x, 'y': rescue.pose.position.y,
                        'z': rescue.pose.position.z,
                    },
                    'orientation': {
                        'x': rescue.pose.orientation.x, 'y': rescue.pose.orientation.y,
                        'z': rescue.pose.orientation.z, 'w': rescue.pose.orientation.w,
                    },
                },
            }
            for index, rescue in enumerate(target.rescue_poses)
        ],
        'observation_count': target.observation_count,
        'confidence': target.confidence,
        'observed_by': sorted(target.observed_by),
        'eligible_for_basic_rescue': True,
    }


def _yaw_from_quaternion(quaternion):
    values = [quaternion[axis] for axis in 'xyzw']
    norm = math.sqrt(sum(value * value for value in values))
    if abs(norm - 1.) > 1e-4:
        raise ValueError('persisted target pose quaternion is not normalized')
    if abs(quaternion['x']) > 1e-6 or abs(quaternion['y']) > 1e-6:
        raise ValueError('persisted target pose must contain yaw only')
    return math.atan2(
        2. * quaternion['w'] * quaternion['z'],
        1. - 2. * quaternion['z'] ** 2,
    )


def _validate_target_semantics(targets, geometry_version, geometry):
    seen_ids = set()
    for target in targets:
        target_id = target['target_id']
        if target_id in seen_ids:
            raise ValueError(f'duplicate target_id {target_id!r}')
        seen_ids.add(target_id)
        yaw = _yaw_from_quaternion(target['marker_pose']['orientation'])
        zones = target['rescue_zones']
        if [zone['index'] for zone in zones] != list(range(len(zones))):
            raise ValueError(f'{target_id}: rescue zone indices must be contiguous from zero')
        if geometry_version is None:
            if zones:
                raise ValueError('rescue zones must be empty without a geometry version')
            continue
        if geometry is None:
            raise ValueError('versioned rescue geometry configuration is required')
        marker_pose = target['marker_pose']
        position = marker_pose['position']
        from .geometry import RigidTransform
        cosine, sine = math.cos(yaw), math.sin(yaw)
        planar_rotation = [
            [cosine, -sine, 0.],
            [sine, cosine, 0.],
            [0., 0., 1.],
        ]
        pose = RigidTransform(planar_rotation, [position['x'], position['y'], position['z']])
        expected = rescue_zone_poses(pose, yaw, target['target_type'], geometry)
        if len(zones) != len(expected):
            raise ValueError(f'{target_id}: incorrect rescue zone count')
        for zone, (index, x, y, z, zone_yaw) in zip(zones, expected):
            zone_pose = zone['pose']
            zone_position = zone_pose['position']
            actual_yaw = _yaw_from_quaternion(zone_pose['orientation'])
            if (zone['index'] != index
                    or abs(zone_position['x'] - x) > 1e-3
                    or abs(zone_position['y'] - y) > 1e-3
                    or abs(zone_position['z'] - z) > 1e-3
                    or abs(normalize_angle(actual_yaw - zone_yaw)) > 1e-3):
                raise ValueError(f'{target_id}: rescue zone does not match configured geometry')


def build_target_list(snapshot, map_metadata, *, rescue_geometry_source,
                      rescue_geometry_version, rescue_geometry=None):
    """Build, Schema-validate, and semantically validate the Schema v2 target document."""
    if not isinstance(snapshot, FrozenTargetSnapshot):
        raise TypeError('snapshot must be a FrozenTargetSnapshot')
    if not isinstance(map_metadata, dict):
        raise TypeError('map_metadata must be a mapping')
    if (map_metadata.get('map_id') != snapshot.map_id
            or map_metadata.get('revision') != snapshot.map_revision
            or map_metadata.get('frame_id') != 'map'):
        raise ValueError('map metadata does not match frozen target snapshot')
    if (rescue_geometry_version is None) != (rescue_geometry_source is None):
        raise ValueError('rescue geometry version and source must both be set or null')
    if rescue_geometry_source not in (None, 'development'):
        raise ValueError('official rescue geometry is not defined by the current project contract')
    if rescue_geometry_source == 'development':
        validate_development_geometry(rescue_geometry, rescue_geometry_version)
    targets = snapshot.targets
    for target in targets:
        target['marker_yaw_rad'] = _yaw_from_quaternion(
            target['marker_pose']['orientation'])
    _validate_target_semantics(targets, rescue_geometry_version, rescue_geometry)
    document = {
        'schema_version': 2,
        'run_id': snapshot.run_id,
        'revision': snapshot.target_revision,
        'frozen_at': snapshot.frozen_at,
        'frame_id': 'map',
        'map': deepcopy(map_metadata),
        'rescue_geometry': {
            'version': rescue_geometry_version,
            'source': rescue_geometry_source,
        },
        'targets': targets,
    }
    _target_list_validator().validate(document)
    return document


def _target_list_validator():
    schema_path = Path(__file__).resolve().parents[3] / 'schemas' / 'target_list.schema.json'
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    return jsonschema.Draft202012Validator(
        schema, format_checker=jsonschema.FormatChecker())


def write_target_list_atomic(document, destination):
    """Write a validated Schema v2 document using same-directory atomic replacement."""
    _target_list_validator().validate(document)
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    serialized = (json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', dir=destination.parent,
                prefix=f'.{destination.name}.', suffix='.tmp', delete=False) as stream:
            temporary_path = Path(stream.name)
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, destination)
        directory_fd = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except OSError:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise
