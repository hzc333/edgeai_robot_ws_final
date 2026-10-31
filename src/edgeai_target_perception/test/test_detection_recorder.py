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
"""Tests for the midterm detection evidence recorder."""

import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

from edgeai_robot_interfaces.msg import TargetObservation

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_target_perception.detection_recorder import (  # noqa: E402
    DetectionRecorder,
    observation_record,
    project_marker_outline,
)
from geometry_msgs.msg import Pose  # noqa: E402
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

K = [554.38, 0.0, 320.5, 0.0, 554.38, 240.5, 0.0, 0.0, 1.0]


def _observation(sec=12, marker_id=2, target_id='C1', x=0.1, y=-0.2, z=0.9):
    message = TargetObservation()
    message.header.stamp.sec = sec
    message.header.frame_id = 'robot_1/camera_optical_frame'
    message.run_id, message.robot_id = 'run-test', 'robot_1'
    message.marker_id, message.target_id = marker_id, target_id
    message.target_type = TargetObservation.TYPE_COOPERATIVE
    pose = message.marker_pose.pose
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    message.confidence = 0.95
    return message


class PureFunctionTests(unittest.TestCase):
    def test_record_has_the_midterm_fields(self):
        record = observation_record(_observation())
        self.assertEqual(record['marker_id'], 2)
        self.assertEqual(record['target_id'], 'C1')
        self.assertEqual(record['target_type'], 'cooperative')
        self.assertEqual(record['sim_time'], 12.0)
        self.assertEqual(record['frame_id'], 'robot_1/camera_optical_frame')
        self.assertEqual(record['position_m'], [0.1, -0.2, 0.9])
        self.assertAlmostEqual(record['range_m'], math.sqrt(0.01 + 0.04 + 0.81), places=4)

    def test_outline_projects_a_facing_marker_around_its_centre(self):
        pose = Pose()
        pose.position.z = 1.0
        # Upright marker facing the camera: 180 deg about x (z_M = -z_cam, y_M = -y_cam).
        pose.orientation.x, pose.orientation.w = 1.0, 0.0
        corners = project_marker_outline(pose, K, 0.16)
        self.assertEqual(len(corners), 4)
        us = [u for u, _ in corners]
        vs = [v for _, v in corners]
        self.assertAlmostEqual(max(us) - min(us), 554.38 * 0.16, places=3)
        self.assertAlmostEqual((max(us) + min(us)) / 2, 320.5, places=3)
        self.assertLess(corners[0][1], corners[3][1])     # TL above BL (y down)
        self.assertAlmostEqual((max(vs) + min(vs)) / 2, 240.5, places=3)

    def test_outline_behind_the_camera_is_none(self):
        pose = Pose()
        pose.position.z = -1.0
        pose.orientation.w = 1.0
        self.assertIsNone(project_marker_outline(pose, K, 0.16))


class RecorderNodeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.directory = tempfile.TemporaryDirectory()
        self.node = DetectionRecorder(parameter_overrides=[
            Parameter('output_dir', value=self.directory.name),
            Parameter('record_period_s', value=1.0)])

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        self.directory.cleanup()

    def test_records_are_throttled_and_summarised_per_target(self):
        for sec in (10, 10, 11, 13):
            self.node._on_observation(_observation(sec=sec))
        self.node._on_observation(_observation(sec=13, marker_id=1, target_id='S1'))
        run_dir = Path(self.directory.name) / 'run-test'
        lines = (run_dir / 'detections.jsonl').read_text().splitlines()
        self.assertEqual([json.loads(line)['sim_time'] for line in lines],
                         [10.0, 11.0, 13.0, 13.0])
        summary = json.loads((run_dir / 'summary.json').read_text())
        self.assertEqual(set(summary['targets']), {'C1', 'S1'})
        self.assertEqual(summary['targets']['C1']['first']['sim_time'], 10.0)
        self.assertEqual(summary['targets']['C1']['latest']['sim_time'], 13.0)
        self.assertEqual(summary['targets']['C1']['records'], 3)


if __name__ == '__main__':
    unittest.main()
