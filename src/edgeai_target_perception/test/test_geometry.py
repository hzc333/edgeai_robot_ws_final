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

"""Synthetic tests for RGB-D validation, camera geometry and rigid transforms."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import cv2

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_target_perception.aruco_core import ArucoCore  # noqa: E402
from edgeai_target_perception.geometry import (  # noqa: E402
    back_project_pixels,
    depth_to_meters,
    marker_to_target_pose,
    RigidTransform,
)
import numpy as np  # noqa: E402


class RgbdGeometryTests(unittest.TestCase):
    def setUp(self):
        self.camera = np.array([[800., 0, 320], [0, 800., 240], [0, 0, 1]])
        self.core = ArucoCore('DICT_4X4_50', .15, {
            1: {'target_id': 'T1', 'type': 'normal'},
        })
        marker_rotation = cv2.Rodrigues(np.array([.12, -.16, .03]))[0]
        self.rotation = cv2.Rodrigues(marker_rotation @ np.diag([1., -1., -1.]))[0].reshape(3)
        self.translation = np.array([.02, -.01, .8])
        self.corners = cv2.projectPoints(
            self.core.object_points(), self.rotation, self.translation,
            self.camera, None)[0].reshape(4, 2)

    def test_depth_unit_conversion_and_invalid_samples(self):
        depth = np.array([[0, 500, 1200], [-1, 2500, 65535]], dtype=np.int32)
        meters = depth_to_meters(depth, unit_scale_m=.001)
        np.testing.assert_allclose(meters[0, 1:], [.5, 1.2])
        np.testing.assert_allclose(meters[1, 1:], [2.5, 65.535])
        self.assertTrue(np.isnan(meters[0, 0]))
        self.assertTrue(np.isnan(meters[1, 0]))
        floats = depth_to_meters(np.array([[np.nan, np.inf, .2]], dtype=np.float32))
        self.assertTrue(np.isnan(floats[0, 0]))
        self.assertTrue(np.isnan(floats[0, 1]))
        self.assertAlmostEqual(floats[0, 2], .2)
        for invalid in ([], np.zeros((2, 2, 2)), np.array(['1'], dtype=object)):
            with self.subTest(invalid=np.asarray(invalid).shape), self.assertRaises(ValueError):
                depth_to_meters(invalid)
        with self.assertRaises(ValueError):
            depth_to_meters(np.ones((2, 2)), unit_scale_m=0)

    def test_back_projection_with_and_without_lens_distortion(self):
        points_3d = np.array([[.1, -.05, .8], [-.15, .08, 1.2]])
        distortion = np.array([.12, -.04, .001, -.002, .01])
        pixels = cv2.projectPoints(
            points_3d, np.zeros(3), np.zeros(3), self.camera, distortion)[0].reshape(-1, 2)
        projected = back_project_pixels(
            pixels, points_3d[:, 2], self.camera, distortion)
        np.testing.assert_allclose(projected, points_3d, atol=1e-9)

        undistorted_pixels = cv2.projectPoints(
            points_3d, np.zeros(3), np.zeros(3), self.camera, None)[0].reshape(-1, 2)
        np.testing.assert_allclose(
            back_project_pixels(undistorted_pixels, points_3d[:, 2], self.camera),
            points_3d, atol=1e-12)
        with self.assertRaises(ValueError):
            back_project_pixels([[320, 240]], 0, self.camera)

    def test_rigid_transform_inverse_and_composition(self):
        first = RigidTransform.from_rvec_tvec([.2, -.1, .3], [.5, -.2, .1])
        second = RigidTransform.from_rvec_tvec([-.1, .15, .05], [.1, .3, -.2])
        point = np.array([.2, -.4, 1.])
        composed = first.compose(second)
        np.testing.assert_allclose(
            composed.transform_points(point),
            first.transform_points(second.transform_points(point)), atol=1e-12)
        np.testing.assert_allclose(first.inverse().transform_points(
            first.transform_points(point)), point, atol=1e-12)
        np.testing.assert_allclose(
            RigidTransform.from_matrix(first.to_matrix()).to_matrix(),
            first.to_matrix(), atol=1e-12)
        with self.assertRaises(ValueError):
            RigidTransform(np.diag([1., 1., -1.]), [0, 0, 0])

    def test_marker_to_target_coordinate_contract(self):
        marker_pose = RigidTransform.from_rvec_tvec(self.rotation, self.translation)
        target_pose = marker_to_target_pose(marker_pose)
        np.testing.assert_allclose(target_pose.rotation, marker_pose.rotation @ np.array([
            [0., 1., 0.], [0., 0., 1.], [1., 0., 0.]]), atol=1e-12)
        np.testing.assert_allclose(target_pose.translation, self.translation)
        np.testing.assert_allclose(target_pose.rotation.T @ target_pose.rotation,
                                   np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(target_pose.rotation), 1.)

    def test_depth_consistency_accepts_registered_depth_in_millimetres(self):
        depth = np.full((480, 640), 800, dtype=np.uint16)
        result = self.core.estimate_pose(
            1, self.corners, self.camera, depth_image=depth,
            depth_scale_m=.001, depth_tolerance_m=.05)
        self.assertTrue(result.valid)
        self.assertIsNotNone(result.depth_consistency_error_m)
        self.assertLess(result.depth_consistency_error_m, .05)

    def test_depth_consistency_rejects_mismatch(self):
        depth = np.full((480, 640), 1200, dtype=np.uint16)
        result = self.core.estimate_pose(
            1, self.corners, self.camera, depth_image=depth,
            depth_scale_m=.001, depth_tolerance_m=.02)
        self.assertFalse(result.valid)
        self.assertEqual(result.rejection_reason, 'depth_inconsistent')

    def test_depth_requires_explicit_scale_and_reports_unavailable_samples(self):
        with self.assertRaises(ValueError):
            self.core.estimate_pose(1, self.corners, self.camera,
                                    depth_image=np.full((480, 640), 800, dtype=np.uint16))
        invalid_depth = np.zeros((480, 640), dtype=np.uint16)
        result = self.core.estimate_pose(
            1, self.corners, self.camera, depth_image=invalid_depth, depth_scale_m=.001)
        self.assertFalse(result.valid)
        self.assertEqual(result.rejection_reason, 'depth_unavailable')

    def test_detection_requires_registered_depth_dimensions(self):
        image = np.full((480, 640), 255, dtype=np.uint8)
        with self.assertRaisesRegex(ValueError, 'registered'):
            self.core.detect(image, self.camera,
                             depth_image=np.ones((479, 640), dtype=np.uint16),
                             depth_scale_m=.001)

    def test_depth_can_select_consistent_candidate_over_pixel_best(self):
        alternate_translation = self.translation + [0., 0., .01]
        alternate_corners = cv2.projectPoints(
            self.core.object_points(), self.rotation, alternate_translation,
            self.camera, None)[0].reshape(4, 2)
        depth = np.full((480, 640), 800, dtype=np.uint16)
        with patch('cv2.solvePnPGeneric', return_value=(
                2, (self.rotation, self.rotation),
                (alternate_translation, self.translation))):
            result = self.core.estimate_pose(
                1, alternate_corners, self.camera, depth_image=depth,
                depth_scale_m=.001, depth_tolerance_m=.005)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.tvec, self.translation, atol=1e-12)

    def test_planar_pnp_ambiguity_diagnostics_are_reported(self):
        result = self.core.estimate_pose(1, self.corners, self.camera)
        self.assertTrue(result.valid)
        self.assertIsNotNone(result.ambiguity)
        self.assertGreaterEqual(result.ambiguity.candidate_count, 1)
        if result.ambiguity.candidate_count > 1:
            self.assertIsNotNone(result.ambiguity.reprojection_error_gap_px)
            self.assertIsNotNone(result.ambiguity.translation_separation_m)
            self.assertIsNotNone(result.ambiguity.rotation_separation_rad)


if __name__ == '__main__':
    unittest.main()
