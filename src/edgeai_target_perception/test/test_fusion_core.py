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

"""Synthetic tests for timestamped coordinate transforms and target fusion."""

import unittest

from edgeai_target_perception.fusion_core import (
    FusionObservation,
    TargetRegistry,
    transform_observation,
)
from edgeai_target_perception.geometry import RigidTransform
import numpy as np

RUN_MAPPING = {
    1: {'target_id': 'T1', 'type': 'normal'},
    10: {'target_id': 'TC1', 'type': 'cooperative'},
}
NOW_NS = 10_000_000_000


def _observation(robot_id='robot_1', position=(1., 2., 0.),
                 stamp_ns=NOW_NS, confidence=.8, marker_id=1,
                 target_id='T1', target_type='normal', variance=.04,
                 rotation=None, covariance=None):
    return FusionObservation(
        stamp_ns=stamp_ns,
        robot_id=robot_id,
        marker_id=marker_id,
        target_id=target_id,
        target_type=target_type,
        position=np.asarray(position, dtype=float),
        rotation=np.eye(3) if rotation is None else rotation,
        covariance=(np.diag([variance] * 3 + [.1] * 3)
                    if covariance is None else covariance),
        confidence=confidence,
    )


class FusionCoreTests(unittest.TestCase):
    def setUp(self):
        self.registry = TargetRegistry(
            RUN_MAPPING, minimum_confidence=.6, position_gate_m=.5,
            stale_after_ns=2_000_000_000)

    def test_coordinate_transform_rotates_pose_and_covariance(self):
        covariance = np.diag([.01, .04, .09, .1, .2, .3])
        source = _observation(position=(1., 0., 0.), covariance=covariance)
        c, s = np.cos(np.pi / 2), np.sin(np.pi / 2)
        transform = RigidTransform(
            np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]),
            np.array([2., 3., 4.]))
        result = transform_observation(source, transform)
        np.testing.assert_allclose(result.position, [2., 4., 4.], atol=1e-12)
        np.testing.assert_allclose(result.rotation, transform.rotation, atol=1e-12)
        np.testing.assert_allclose(
            np.diag(result.covariance), [.04, .01, .09, .2, .1, .3], atol=1e-12)
        self.assertEqual(result.stamp_ns, source.stamp_ns)

    def test_repeated_frames_replace_robot_evidence_without_double_counting(self):
        first = _observation(stamp_ns=NOW_NS - 100_000_000, confidence=.8)
        repeated = _observation(stamp_ns=NOW_NS - 50_000_000, confidence=.9)
        self.assertTrue(self.registry.ingest(first, NOW_NS))
        self.assertFalse(self.registry.ingest(first, NOW_NS))
        self.assertTrue(self.registry.ingest(repeated, NOW_NS))
        estimate, = self.registry.snapshot()
        self.assertEqual(estimate.observation_count, 1)
        self.assertEqual(estimate.observed_by, ('robot_1',))
        self.assertAlmostEqual(estimate.confidence, .9)
        self.assertEqual(estimate.stamp_ns, repeated.stamp_ns)

    def test_multi_robot_fusion_uses_confidence_and_covariance(self):
        first = _observation(
            robot_id='robot_1', position=(0., 0., 1.), confidence=.8, variance=.04)
        second = _observation(
            robot_id='robot_2', position=(.1, 0., 1.), confidence=.9, variance=.01,
            stamp_ns=NOW_NS - 10_000_000)
        self.assertTrue(self.registry.ingest(first, NOW_NS))
        self.assertTrue(self.registry.ingest(second, NOW_NS))
        estimate, = self.registry.snapshot()
        expected_x = (.8 / .04 * 0. + .9 / .01 * .1) / (.8 / .04 + .9 / .01)
        self.assertAlmostEqual(estimate.position[0], expected_x)
        self.assertEqual(estimate.observation_count, 2)
        self.assertEqual(estimate.observed_by, ('robot_1', 'robot_2'))
        self.assertAlmostEqual(estimate.confidence, .98)
        self.assertGreaterEqual(estimate.covariance[0, 0], .01)

    def test_orientation_spread_is_added_to_fused_covariance(self):
        yaw = .2
        rotation = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.],
            [np.sin(yaw), np.cos(yaw), 0.],
            [0., 0., 1.],
        ])
        self.assertTrue(self.registry.ingest(
            _observation(robot_id='robot_1', position=(0., 0., 1.)), NOW_NS))
        self.assertTrue(self.registry.ingest(
            _observation(robot_id='robot_2', position=(0., 0., 1.),
                         rotation=rotation, stamp_ns=NOW_NS - 1), NOW_NS))
        estimate, = self.registry.snapshot()
        self.assertGreater(estimate.covariance[5, 5], .1)

    def test_spatial_outlier_is_rejected_without_registry_change(self):
        self.assertTrue(self.registry.ingest(_observation(), NOW_NS))
        revision = self.registry.revision
        outlier = _observation(
            robot_id='robot_2', position=(2., 2., 0.), stamp_ns=NOW_NS - 1)
        self.assertFalse(self.registry.ingest(outlier, NOW_NS))
        self.assertEqual(self.registry.revision, revision)
        estimate, = self.registry.snapshot()
        self.assertEqual(estimate.observed_by, ('robot_1',))

    def test_unknown_or_inconsistent_marker_identity_is_rejected(self):
        self.assertFalse(self.registry.ingest(
            _observation(marker_id=99, target_id='T99'), NOW_NS))
        self.assertFalse(self.registry.ingest(
            _observation(marker_id=1, target_id='T2'), NOW_NS))
        self.assertFalse(self.registry.ingest(
            _observation(marker_id=10, target_id='TC1'), NOW_NS))
        self.assertEqual(self.registry.snapshot(), ())

    def test_configured_cooperative_identity_is_accepted(self):
        observation = _observation(
            marker_id=10, target_id='TC1', target_type='cooperative')
        self.assertTrue(self.registry.ingest(observation, NOW_NS))
        estimate, = self.registry.snapshot()
        self.assertEqual(estimate.target_type, 'cooperative')

    def test_stale_evidence_is_rejected_and_pruned(self):
        stale = _observation(stamp_ns=NOW_NS - 3_000_000_000)
        self.assertFalse(self.registry.ingest(stale, NOW_NS))
        fresh = _observation()
        self.assertTrue(self.registry.ingest(fresh, NOW_NS))
        self.assertTrue(self.registry.prune(NOW_NS + 2_000_000_001))
        self.assertEqual(self.registry.snapshot(), ())

    def test_reset_isolates_runs_and_restarts_registry_version(self):
        self.assertTrue(self.registry.ingest(_observation(), NOW_NS))
        self.assertEqual(self.registry.revision, 1)
        self.registry.reset()
        self.assertEqual(self.registry.revision, 0)
        self.assertEqual(self.registry.snapshot(), ())
        self.assertTrue(self.registry.ingest(
            _observation(stamp_ns=NOW_NS + 1), NOW_NS + 1))
        self.assertEqual(self.registry.revision, 1)

    def test_pose_covariance_and_confidence_are_validated(self):
        with self.assertRaises(ValueError):
            _observation(covariance=np.diag([1., 1., 1., 1., 1., -1.]))
        with self.assertRaises(ValueError):
            _observation(confidence=1.1)


if __name__ == '__main__':
    unittest.main()
