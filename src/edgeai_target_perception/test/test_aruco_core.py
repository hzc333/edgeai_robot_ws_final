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

"""Standalone synthetic-image and pose-contract tests; run with unittest discovery."""

import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_target_perception.aruco_core import ArucoCore  # noqa: E402
import numpy as np  # noqa: E402


class ArucoCoreTests(unittest.TestCase):
    def setUp(self):
        self.core = ArucoCore('DICT_4X4_50', 0.15, {
            1: {'target_id': 'T1', 'type': 'normal'},
            10: {'target_id': 'TC1', 'type': 'cooperative'},
        })
        self.camera = np.array([[800., 0, 320], [0, 800., 240], [0, 0, 1]])
        # Front face towards camera: M y points up and M z points to viewer.
        tilt = cv2.Rodrigues(np.array([0.15, -0.20, 0.08]))[0]
        self.rotation = cv2.Rodrigues(tilt @ np.diag([1., -1., -1.]))[0].reshape(3)
        self.translation = np.array([0.03, -0.02, 0.8])

    def project(self, rotation=None, translation=None, distortion=None):
        return cv2.projectPoints(
            self.core.object_points(), self.rotation if rotation is None else rotation,
            self.translation if translation is None else translation,
            self.camera, distortion)[0].reshape(4, 2)

    def render(self, marker_id=1, rotation=None, translation=None):
        marker = cv2.aruco.generateImageMarker(self.core.dictionary, marker_id, 300)
        source = np.float32([[0, 0], [299, 0], [299, 299], [0, 299]])
        destination = self.project(rotation, translation).astype(np.float32)
        homography = cv2.getPerspectiveTransform(source, destination)
        return cv2.warpPerspective(marker, homography, (640, 480),
                                   flags=cv2.INTER_NEAREST, borderValue=255)

    def test_geometry_contract(self):
        np.testing.assert_allclose(self.core.object_points(), [
            [-.075, .075, 0], [.075, .075, 0], [.075, -.075, 0], [-.075, -.075, 0]])

    def test_exact_pose_with_distortion(self):
        distortion = np.array([.12, -.04, .001, -.002, .01])
        result = self.core.estimate_pose(1, self.project(distortion=distortion),
                                         self.camera, distortion)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(result.tvec, self.translation, atol=1e-7)
        np.testing.assert_allclose(cv2.Rodrigues(result.rvec)[0],
                                   cv2.Rodrigues(self.rotation)[0], atol=1e-7)
        self.assertLess(result.reprojection_error_px, 1e-7)

    def test_synthetic_images_at_distances_and_angles(self):
        for depth in (0.5, 0.8, 1.5):
            for tilt in (0.1, 0.4):
                with self.subTest(depth=depth, tilt=tilt):
                    rotation = cv2.Rodrigues(
                        cv2.Rodrigues(np.array([tilt, -.15, .1]))[0]
                        @ np.diag([1., -1., -1.]))[0].reshape(3)
                    translation = np.array([.03, -.02, depth])
                    results = self.core.detect(
                        self.render(rotation=rotation, translation=translation), self.camera)
                    self.assertEqual(len(results), 1)
                    result = results[0]
                    self.assertTrue(result.valid)
                    self.assertEqual(result.identity.target_id, 'T1')
                    np.testing.assert_allclose(result.tvec, translation, atol=depth * .025)
                    relative = cv2.Rodrigues(result.rvec)[0] @ cv2.Rodrigues(rotation)[0].T
                    angle = np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1))
                    self.assertLess(angle, .12)
                    self.assertLess(result.reprojection_error_px, 1.)

    def test_cooperative_identity(self):
        result = self.core.detect(self.render(marker_id=10), self.camera)[0]
        self.assertTrue(result.valid)
        self.assertEqual(result.identity.target_type, 'cooperative')

    def test_rotated_marker_corner_order(self):
        rotation = cv2.Rodrigues(
            cv2.Rodrigues(np.array([0., 0., np.pi / 2]))[0]
            @ cv2.Rodrigues(self.rotation)[0])[0].reshape(3)
        result = self.core.detect(self.render(rotation=rotation), self.camera)[0]
        self.assertTrue(result.valid)
        np.testing.assert_allclose(cv2.Rodrigues(result.rvec)[0],
                                   cv2.Rodrigues(rotation)[0], atol=.06)

    def test_multiple_known_and_unknown_markers(self):
        left = self.render(1, translation=np.array([-.17, 0., .8]))
        right = self.render(7, translation=np.array([.17, 0., .8]))
        results = self.core.detect(np.minimum(left, right), self.camera)
        self.assertEqual([result.marker_id for result in results], [1, 7])
        self.assertTrue(results[0].valid)
        self.assertFalse(results[1].valid)

    def test_resolved_configuration_adapter(self):
        values = {'aruco.dictionary': 'DICT_5X5_100', 'aruco.marker_size_m': .2,
                  'aruco.id_mapping': {42: {'target_id': 'custom', 'type': 'cooperative'}}}
        core = ArucoCore.from_resolved_parameters(SimpleNamespace(require=values.__getitem__))
        self.assertEqual(core.marker_size_m, .2)
        self.assertEqual(set(core.id_mapping), {42})
        marker = cv2.aruco.generateImageMarker(core.dictionary, 42, 160)
        image = np.full((240, 240), 255, np.uint8)
        image[40:200, 40:200] = marker
        result = core.detect(image, self.camera)[0]
        self.assertTrue(result.valid)
        self.assertEqual(result.identity.target_id, 'custom')

    def test_unknown_marker_is_diagnostic_only(self):
        result = self.core.detect(self.render(marker_id=7), self.camera)[0]
        self.assertFalse(result.valid)
        self.assertIsNone(result.identity)
        self.assertIsNone(result.tvec)
        self.assertEqual(result.rejection_reason, 'unknown_marker_id')

    def test_blank_image(self):
        self.assertEqual(self.core.detect(np.full((480, 640), 255, np.uint8), self.camera), [])

    def test_blurred_bgr_image(self):
        image = cv2.GaussianBlur(self.render(), (3, 3), .7)
        result = self.core.detect(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), self.camera)[0]
        self.assertTrue(result.valid)

    def test_reprojection_rms(self):
        points = self.project()
        points[0] += [2, -1]
        result = self.core.estimate_pose(1, points, self.camera)
        projected = cv2.projectPoints(self.core.object_points(), result.rvec, result.tvec,
                                      self.camera, None)[0].reshape(4, 2)
        expected = np.sqrt(np.mean(np.sum((points - projected) ** 2, axis=1)))
        self.assertAlmostEqual(result.reprojection_error_px, expected)
        self.assertGreater(expected, 0)
        self.core.max_reprojection_error_px = expected / 2
        self.assertEqual(self.core.estimate_pose(1, points, self.camera).rejection_reason,
                         'reprojection_error_exceeded')

    def test_negative_depth_candidates_rejected(self):
        with patch('cv2.solvePnPGeneric', return_value=(
                1, (self.rotation.reshape(3, 1),), (np.array([[0.], [0.], [-1.]]),))):
            result = self.core.estimate_pose(1, self.project(), self.camera)
        self.assertFalse(result.valid)
        self.assertEqual(result.rejection_reason, 'no_positive_depth_solution')

    def test_corner_behind_camera_rejected_even_when_center_positive(self):
        with patch('cv2.solvePnPGeneric', return_value=(
                1, (np.array([[0.], [np.pi / 2], [0.]]),),
                (np.array([[0.], [0.], [.01]]),))):
            result = self.core.estimate_pose(1, self.project(), self.camera)
        self.assertEqual(result.rejection_reason, 'no_positive_depth_solution')

    def test_candidate_with_lowest_error_selected(self):
        wrong = self.translation + [.1, 0, 0]
        with patch('cv2.solvePnPGeneric', return_value=(
                2, (self.rotation, self.rotation), (wrong, self.translation))):
            result = self.core.estimate_pose(1, self.project(), self.camera)
        np.testing.assert_allclose(result.tvec, self.translation)

    def test_degenerate_corners(self):
        result = self.core.estimate_pose(1, np.zeros((4, 2)), self.camera)
        self.assertEqual(result.rejection_reason, 'degenerate_corners')

    def test_solver_failure(self):
        with patch('cv2.solvePnPGeneric', return_value=(0, (), ())):
            result = self.core.estimate_pose(1, self.project(), self.camera)
        self.assertEqual(result.rejection_reason, 'pose_estimation_failed')
        with patch('cv2.solvePnPGeneric', side_effect=cv2.error('solver error')):
            result = self.core.estimate_pose(1, self.project(), self.camera)
        self.assertEqual(result.rejection_reason, 'pose_estimation_failed')

    def test_invalid_inputs(self):
        for image in (np.zeros((2, 2), dtype=float), np.zeros((0, 2), np.uint8),
                      np.zeros((2, 2, 4), np.uint8)):
            with self.subTest(image_shape=image.shape), self.assertRaises(ValueError):
                self.core.detect(image, self.camera)
        for matrix in (np.eye(2), np.zeros((3, 3)), np.full((3, 3), np.nan)):
            with self.assertRaises(ValueError):
                self.core.detect(self.render(), matrix)
        with self.assertRaises(ValueError):
            self.core.estimate_pose(1, self.project(), self.camera, [1, 2, 3])
        with self.assertRaises(ValueError):
            self.core.estimate_pose(1, np.full((4, 2), np.nan), self.camera)

    def test_invalid_configuration(self):
        for dictionary, size, mapping in (
                ('invalid', .15, {1: {}}), ('DICT_4X4_50', -.1, {1: {}}),
                ('DICT_4X4_50', .15, {}),
                ('DICT_4X4_50', .15, {1: {'target_id': 'T1', 'type': 'unknown'}})):
            with self.assertRaises(ValueError):
                ArucoCore(dictionary, size, mapping)


@unittest.skipUnless(importlib.util.find_spec('yaml'), 'PyYAML is not installed')
class RepositoryConfigTests(unittest.TestCase):
    def test_actual_development_configuration(self):
        core = ArucoCore.from_repository_config(config_dir=ROOT / 'config')
        self.assertEqual(core.marker_size_m, .15)
        self.assertEqual(core.id_mapping[10].target_id, 'TC1')
        self.assertEqual(set(core.id_mapping), {1, 2, 3, 10})

    def test_competition_does_not_fall_back(self):
        from edgeai_common.config import MissingOfficialParameters
        with self.assertRaises(MissingOfficialParameters):
            ArucoCore.from_repository_config('competition', ROOT / 'config')


if __name__ == '__main__':
    unittest.main()
