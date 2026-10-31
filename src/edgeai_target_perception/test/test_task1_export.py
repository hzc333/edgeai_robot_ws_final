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

"""Tests for immutable Task 1 snapshots and Schema v2 atomic target export."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_robot_interfaces.msg import CompetitionState, Target, TargetArray  # noqa: E402
from edgeai_target_perception.geometry import RigidTransform  # noqa: E402
from edgeai_target_perception.rescue_geometry import rescue_zone_poses  # noqa: E402
from edgeai_target_perception.task1_export import (  # noqa: E402
    build_target_list,
    SnapshotConflictError,
    Task1SnapshotFreezer,
    write_target_list_atomic,
)
import jsonschema  # noqa: E402


FROZEN_AT = '2026-10-25T14:00:00Z'
GEOMETRY = {
    'version': 'development-v1',
    'zone_orientation': 'face_marker',
    'normal': {'forward_offset_m': .5},
    'cooperative': {
        'forward_offset_m': .5,
        'lateral_offsets_m': [-.3, .3],
    },
}
MAP_METADATA = {
    'map_id': 'map-a',
    'revision': 1,
    'frame_id': 'map',
    'yaml': 'task1/map.yaml',
    'yaml_sha256': 'a' * 64,
    'image': 'task1/map.pgm',
    'image_sha256': 'b' * 64,
    'resolution_m': .05,
    'origin': {'x': -2.6, 'y': -2.6, 'yaw': 0.},
    'width_cells': 104,
    'height_cells': 104,
}


def _target(target_id, marker_id, target_type):
    target = Target()
    target.header.frame_id = 'map'
    target.run_id = 'run-a'
    target.map_id = 'map-a'
    target.map_revision = 1
    target.target_id = target_id
    target.marker_id = marker_id
    target.target_type = (
        Target.TYPE_NORMAL if target_type == 'normal' else Target.TYPE_COOPERATIVE)
    target.marker_pose.pose.position.x = 1.3 if target_type == 'normal' else 3.
    target.marker_pose.pose.position.y = 2.1 if target_type == 'normal' else .4
    target.marker_pose.pose.position.z = .2
    target.marker_pose.pose.orientation.w = 1.
    target.marker_pose.covariance[0] = .01
    target.marker_pose.covariance[7] = .01
    target.marker_pose.covariance[14] = .02
    target.marker_pose.covariance[21] = .05
    target.marker_pose.covariance[28] = .05
    target.marker_pose.covariance[35] = .03
    target.observation_count = 2
    target.confidence = .9
    target.observed_by = ['robot_2', 'robot_1']
    target_pose = RigidTransform(
        [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
        [target.marker_pose.pose.position.x, target.marker_pose.pose.position.y, .2])
    yaw = 0.
    for index, x, y, z, zone_yaw in rescue_zone_poses(
            target_pose, yaw, target_type, GEOMETRY):
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(zone_yaw / 2)
        pose.pose.orientation.w = math.cos(zone_yaw / 2)
        target.rescue_poses.append(pose)
    return target


def _array(run_id='run-a', revision=2, targets=None):
    array = TargetArray()
    array.header.frame_id = 'map'
    array.run_id = run_id
    array.map_id = 'map-a'
    array.map_revision = 1
    array.revision = revision
    array.task1_frozen = False
    array.targets = [] if targets is None else targets
    return array


def _state(run_id='run-a', target_revision=2):
    state = CompetitionState()
    state.run_id = run_id
    state.map_id = 'map-a'
    state.map_revision = 1
    state.target_revision = target_revision
    state.phase = CompetitionState.PHASE_TASK1
    return state


def _snapshot(targets=None, *, run_id='run-a', revision=2):
    return Task1SnapshotFreezer().freeze(
        _array(run_id, revision, targets),
        _state(run_id, revision),
        run_id=run_id,
        map_id='map-a',
        expected_map_revision=1,
        expected_target_revision=revision,
        frozen_at=FROZEN_AT,
    )


class Task1ExportTests(unittest.TestCase):
    def test_freeze_is_immutable_and_duplicate_request_is_idempotent(self):
        source = _array(targets=[_target('T1', 1, 'normal')])
        freezer = Task1SnapshotFreezer()
        first = freezer.freeze(
            source, _state(), run_id='run-a', map_id='map-a',
            expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)
        original_json = first.targets_json
        source.targets[0].target_id = 'T2'
        retry = freezer.freeze(
            source, _state(), run_id='run-a', map_id='map-a',
            expected_map_revision=1, expected_target_revision=2,
            frozen_at='2026-10-25T14:01:00Z')
        self.assertIs(first, retry)
        self.assertEqual(retry.targets_json, original_json)
        self.assertEqual(retry.targets[0]['target_id'], 'T1')

    def test_freeze_rejects_version_mismatch_and_same_run_revision_change(self):
        freezer = Task1SnapshotFreezer()
        with self.assertRaisesRegex(ValueError, 'does not match'):
            freezer.freeze(
                _array(revision=3), _state(), run_id='run-a', map_id='map-a',
                expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)
        freezer.freeze(
            _array(), _state(), run_id='run-a', map_id='map-a',
            expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)
        with self.assertRaises(SnapshotConflictError):
            freezer.freeze(
                _array(revision=3), _state(target_revision=3), run_id='run-a', map_id='map-a',
                expected_map_revision=1, expected_target_revision=3, frozen_at=FROZEN_AT)

    def test_freeze_rejects_embedded_target_metadata_mismatches(self):
        mutations = {
            'frame': lambda target: setattr(target.header, 'frame_id', 'odom'),
            'run': lambda target: setattr(target, 'run_id', 'run-b'),
            'map': lambda target: setattr(target, 'map_id', 'map-b'),
            'map revision': lambda target: setattr(target, 'map_revision', 2),
        }
        for field, mutate in mutations.items():
            with self.subTest(field=field):
                target = _target('T1', 1, 'normal')
                mutate(target)
                with self.assertRaisesRegex(ValueError, 'metadata does not match'):
                    Task1SnapshotFreezer().freeze(
                        _array(targets=[target]), _state(),
                        run_id='run-a', map_id='map-a',
                        expected_map_revision=1, expected_target_revision=2,
                        frozen_at=FROZEN_AT)

    def test_frozen_or_reset_competition_state_cannot_start_a_snapshot(self):
        state = _state()
        state.frozen = True
        with self.assertRaisesRegex(ValueError, 'does not match'):
            Task1SnapshotFreezer().freeze(
                _array(), state, run_id='run-a', map_id='map-a',
                expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)
        state = _state()
        state.phase = CompetitionState.PHASE_TASK2A
        with self.assertRaisesRegex(ValueError, 'does not match'):
            Task1SnapshotFreezer().freeze(
                _array(), state, run_id='run-a', map_id='map-a',
                expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)
        state = _state()
        state.reset_in_progress = True
        with self.assertRaisesRegex(ValueError, 'does not match'):
            Task1SnapshotFreezer().freeze(
                _array(), state, run_id='run-a', map_id='map-a',
                expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)

    def test_same_run_map_revision_change_cannot_replace_frozen_targets(self):
        freezer = Task1SnapshotFreezer()
        freezer.freeze(
            _array(), _state(), run_id='run-a', map_id='map-a',
            expected_map_revision=1, expected_target_revision=2, frozen_at=FROZEN_AT)
        changed = _array()
        changed.map_revision = 2
        changed_state = _state()
        changed_state.map_revision = 2
        with self.assertRaises(SnapshotConflictError):
            freezer.freeze(
                changed, changed_state, run_id='run-a', map_id='map-a',
                expected_map_revision=2, expected_target_revision=2, frozen_at=FROZEN_AT)

    def test_new_run_and_empty_target_set_are_supported(self):
        freezer = Task1SnapshotFreezer()
        empty = freezer.freeze(
            _array(revision=0), _state(target_revision=0), run_id='run-a', map_id='map-a',
            expected_map_revision=1, expected_target_revision=0, frozen_at=FROZEN_AT)
        next_run = freezer.freeze(
            _array(run_id='run-b', revision=0),
            _state(run_id='run-b', target_revision=0), run_id='run-b', map_id='map-a',
            expected_map_revision=1, expected_target_revision=0, frozen_at=FROZEN_AT)
        self.assertEqual(empty.targets, [])
        self.assertEqual(next_run.run_id, 'run-b')

    def test_schema_v2_export_sorts_targets_and_validates_geometry(self):
        snapshot = _snapshot([
            _target('TC1', 10, 'cooperative'),
            _target('T1', 1, 'normal'),
        ])
        document = build_target_list(
            snapshot, MAP_METADATA,
            rescue_geometry_source='development',
            rescue_geometry_version='development-v1',
            rescue_geometry=GEOMETRY,
        )
        self.assertEqual(document['schema_version'], 2)
        self.assertEqual([target['target_id'] for target in document['targets']], ['T1', 'TC1'])
        self.assertEqual(len(document['targets'][0]['rescue_zones']), 1)
        self.assertEqual(len(document['targets'][1]['rescue_zones']), 2)
        self.assertTrue(all(
            target['eligible_for_basic_rescue'] for target in document['targets']))

    def test_rescue_zone_geometry_mismatch_is_rejected(self):
        target = _target('T1', 1, 'normal')
        target.rescue_poses[0].pose.position.x += .1
        with self.assertRaisesRegex(ValueError, 'does not match configured geometry'):
            build_target_list(
                _snapshot([target]), MAP_METADATA,
                rescue_geometry_source='development',
                rescue_geometry_version='development-v1',
                rescue_geometry=GEOMETRY,
            )

    def test_empty_export_without_rules_uses_null_geometry_and_schema_v2(self):
        document = build_target_list(
            _snapshot(), MAP_METADATA,
            rescue_geometry_source=None,
            rescue_geometry_version=None,
        )
        self.assertEqual(document['targets'], [])
        self.assertEqual(document['rescue_geometry'], {'version': None, 'source': None})

    def test_empty_export_may_keep_nonzero_registry_revision(self):
        document = build_target_list(
            _snapshot(revision=3), MAP_METADATA,
            rescue_geometry_source=None,
            rescue_geometry_version=None,
        )
        self.assertEqual(document['targets'], [])
        self.assertEqual(document['revision'], 3)

    def test_unknown_official_geometry_is_not_assumed(self):
        with self.assertRaisesRegex(ValueError, 'not defined'):
            build_target_list(
                _snapshot(), MAP_METADATA,
                rescue_geometry_source='official',
                rescue_geometry_version='official-v1')

    def test_map_metadata_and_rule_mismatches_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'map metadata'):
            build_target_list(
                _snapshot(), dict(MAP_METADATA, revision=2),
                rescue_geometry_source=None, rescue_geometry_version=None)
        with self.assertRaisesRegex(ValueError, 'version and source'):
            build_target_list(
                _snapshot(), MAP_METADATA,
                rescue_geometry_source='development', rescue_geometry_version=None)

    def test_atomic_write_validates_schema_and_preserves_previous_file_on_failure(self):
        document = build_target_list(
            _snapshot([_target('T1', 1, 'normal')]), MAP_METADATA,
            rescue_geometry_source='development',
            rescue_geometry_version='development-v1',
            rescue_geometry=GEOMETRY,
        )
        schema = json.loads(
            (ROOT / 'schemas' / 'target_list.schema.json').read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / 'targets.json'
            write_target_list_atomic(document, destination)
            saved = json.loads(destination.read_text(encoding='utf-8'))
            jsonschema.Draft202012Validator(schema).validate(saved)
            original = destination.read_bytes()
            with patch('edgeai_target_perception.task1_export.os.replace',
                       side_effect=OSError('injected atomic replace failure')):
                with self.assertRaisesRegex(OSError, 'injected'):
                    write_target_list_atomic(document, destination)
            self.assertEqual(destination.read_bytes(), original)
            self.assertEqual(list(Path(temporary).glob('.*.tmp')), [])
            with self.assertRaises(jsonschema.ValidationError):
                write_target_list_atomic({'schema_version': 2}, destination)


if __name__ == '__main__':
    unittest.main()
