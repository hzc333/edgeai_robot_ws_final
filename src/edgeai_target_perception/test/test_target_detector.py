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

"""Tests for TargetObservation conversion and detector result safety filtering."""

from pathlib import Path
import sys
import unittest

import cv2
from cv_bridge import CvBridge
from edgeai_robot_interfaces.msg import CompetitionState, TargetObservation

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_target_perception.aruco_core import (  # noqa: E402
    MarkerResult,
    PlanarPnpDiagnostics,
    TargetIdentity,
)
from edgeai_target_perception.target_detector import (  # noqa: E402
    detect_observations,
    observations_from_results,
    TargetDetector,
)
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from sensor_msgs.msg import CameraInfo  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


def _result(*, target_type='normal', ambiguous=False, valid=True, error=.2):
    return MarkerResult(
        marker_id=10,
        corners=np.zeros((4, 2)),
        identity=TargetIdentity('TC1', target_type) if valid else None,
        rvec=np.array([0., 0., np.pi / 2]) if valid else None,
        tvec=np.array([.1, -.2, .8]) if valid else None,
        reprojection_error_px=error if valid else None,
        rejection_reason=None if valid else 'unknown_marker_id',
        ambiguity=PlanarPnpDiagnostics(2, .1, .02, .2, ambiguous) if valid else None,
    )


def _header():
    header = Header()
    header.stamp.sec = 23
    header.stamp.nanosec = 456
    header.frame_id = 'robot_2/camera_optical_frame'
    return header


class TargetDetectorTests(unittest.TestCase):
    def test_target_observation_message_fields_and_pose_conversion(self):
        observation = observations_from_results([_result(target_type='cooperative')],
                                                _header(), 'robot_2', 'run-abc')[0]
        self.assertIsInstance(observation, TargetObservation)
        self.assertEqual(observation.header.stamp.sec, 23)
        self.assertEqual(observation.header.stamp.nanosec, 456)
        self.assertEqual(observation.header.frame_id, 'robot_2/camera_optical_frame')
        self.assertEqual(observation.run_id, 'run-abc')
        self.assertEqual(observation.robot_id, 'robot_2')
        self.assertEqual(observation.marker_id, 10)
        self.assertEqual(observation.target_id, 'TC1')
        self.assertEqual(observation.target_type, TargetObservation.TYPE_COOPERATIVE)
        np.testing.assert_allclose(
            [observation.marker_pose.pose.position.x,
             observation.marker_pose.pose.position.y,
             observation.marker_pose.pose.position.z], [.1, -.2, .8])
        expected = cv2.Rodrigues(np.array([0., 0., np.pi / 2]))[0]
        quaternion = observation.marker_pose.pose.orientation
        recovered_vector = cv2.Rodrigues(np.array([
            [1 - 2 * (quaternion.y ** 2 + quaternion.z ** 2),
             2 * (quaternion.x * quaternion.y - quaternion.z * quaternion.w),
             2 * (quaternion.x * quaternion.z + quaternion.y * quaternion.w)],
            [2 * (quaternion.x * quaternion.y + quaternion.z * quaternion.w),
             1 - 2 * (quaternion.x ** 2 + quaternion.z ** 2),
             2 * (quaternion.y * quaternion.z - quaternion.x * quaternion.w)],
            [2 * (quaternion.x * quaternion.z - quaternion.y * quaternion.w),
             2 * (quaternion.y * quaternion.z + quaternion.x * quaternion.w),
             1 - 2 * (quaternion.x ** 2 + quaternion.y ** 2)],
        ], dtype=np.float64))[0]
        recovered = cv2.Rodrigues(recovered_vector)[0]
        np.testing.assert_allclose(recovered, expected, atol=1e-12)
        covariance = np.asarray(observation.marker_pose.covariance).reshape(6, 6)
        np.testing.assert_array_equal(np.diag(covariance), np.full(6, 1e6))
        self.assertGreaterEqual(observation.confidence, 0)
        self.assertLessEqual(observation.confidence, 1)
        self.assertTrue(observation.detector_version)

    def test_observations_filter_unknown_invalid_and_ambiguous_results(self):
        results = [
            _result(),
            _result(ambiguous=True),
            _result(valid=False),
            _result(target_type='unsupported'),
        ]
        observations = observations_from_results(results, _header(), 'robot_2', 'run-abc')
        self.assertEqual(len(observations), 1)
        self.assertEqual(observations[0].target_type, TargetObservation.TYPE_NORMAL)

    def test_observation_conversion_requires_valid_identity_and_run_context(self):
        with self.assertRaises(ValueError):
            observations_from_results([_result()], _header(), 'robot_9', 'run-abc')
        with self.assertRaises(ValueError):
            observations_from_results([_result()], _header(), 'robot_1', '')

    def test_detector_calls_geometry_core_and_returns_only_safe_observations(self):
        class FakeCore:
            def __init__(self):
                self.arguments = None

            def detect(self, image, matrix, distortion, **kwargs):
                self.arguments = (image, matrix, distortion, kwargs)
                return [_result(), _result(ambiguous=True), _result(valid=False)]

        core = FakeCore()
        image = np.zeros((20, 30, 3), dtype=np.uint8)
        matrix = np.array([[500., 0, 15], [0, 500., 10], [0, 0, 1]])
        depth = np.full((20, 30), .8)
        results, observations = detect_observations(
            core, image, matrix, np.zeros(5), _header(), 'robot_2', 'run-abc', depth)
        self.assertEqual(len(results), 3)
        self.assertEqual(len(observations), 1)
        self.assertIs(core.arguments[0], image)
        np.testing.assert_array_equal(core.arguments[1], matrix)
        self.assertEqual(core.arguments[2].shape, (5,))
        self.assertIs(core.arguments[3]['depth_image'], depth)
        self.assertEqual(core.arguments[3]['depth_scale_m'], 1.0)

    def test_detector_omits_depth_when_sensor_is_unconfigured(self):
        class FakeCore:
            def detect(self, _image, _matrix, _distortion, **kwargs):
                self_outer.assertEqual(
                    kwargs, {'depth_image': None, 'depth_scale_m': None})
                return [_result()]

        self_outer = self
        _, observations = detect_observations(
            FakeCore(), np.zeros((1, 1), dtype=np.uint8), np.eye(3), None,
            _header(), 'robot_1', 'run-xyz')
        self.assertEqual(len(observations), 1)

    def test_ros_node_converts_camera_messages_and_gates_by_run_state(self):
        started_context = not rclpy.ok()
        if started_context:
            rclpy.init()
        node = None
        try:
            node = TargetDetector()
            node.depth_enabled = True

            class FakePublisher:
                def __init__(self):
                    self.messages = []

                def publish(self, message):
                    self.messages.append(message)

            class FakeCore:
                def detect(self, _image, _matrix, _distortion, **_kwargs):
                    return [_result()]

            publisher = FakePublisher()
            node.observation_pub = publisher
            node.core = FakeCore()
            info = CameraInfo()
            info.header.frame_id = 'robot_1/camera_optical_frame'
            info.width = 8
            info.height = 6
            info.k = [100., 0., 4., 0., 100., 3., 0., 0., 1.]
            node._camera_info_callback(info)

            state = CompetitionState()
            state.run_id = 'run-integration'
            state.phase = state.PHASE_TASK1
            node._competition_callback(state)

            image = CvBridge().cv2_to_imgmsg(np.zeros((6, 8, 3), dtype=np.uint8), 'bgr8')
            image.header = _header()
            image.header.frame_id = 'robot_1/camera_optical_frame'
            image.header.stamp.sec = 29
            node._image_callback(image)
            self.assertEqual(len(publisher.messages), 1)
            self.assertEqual(publisher.messages[0].run_id, 'run-integration')
            self.assertEqual(publisher.messages[0].header.stamp.sec, 29)
            self.assertEqual(publisher.messages[0].header.frame_id,
                             'robot_1/camera_optical_frame')

            state.frozen = True
            node._competition_callback(state)
            node._image_callback(image)
            self.assertEqual(len(publisher.messages), 1)
        finally:
            if node is not None:
                node.destroy_node()
            if started_context:
                rclpy.shutdown()
