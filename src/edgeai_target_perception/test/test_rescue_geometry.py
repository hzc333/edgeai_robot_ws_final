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

"""Tests for map-frame target projection and versioned rescue-zone geometry."""

import math
from pathlib import Path
import sys
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_target_perception.geometry import RigidTransform  # noqa: E402
from edgeai_target_perception.rescue_geometry import (  # noqa: E402
    rescue_zone_poses,
    target_planar_pose,
    validate_development_geometry,
)
import numpy as np  # noqa: E402


GEOMETRY = {
    'version': 'development-v1',
    'zone_orientation': 'face_marker',
    'normal': {'forward_offset_m': .5},
    'cooperative': {
        'forward_offset_m': .5,
        'lateral_offsets_m': [-.3, .3],
    },
}


class RescueGeometryTests(unittest.TestCase):
    def test_marker_pose_projects_to_target_yaw_and_normal_zone(self):
        marker_rotation = np.array([
            [0., 0., 1.],
            [1., 0., 0.],
            [0., 1., 0.],
        ])
        marker = RigidTransform(marker_rotation, [1.3, 2.1, .2])
        target, yaw = target_planar_pose(marker)
        self.assertAlmostEqual(yaw, 0.)
        np.testing.assert_allclose(target.rotation, np.eye(3), atol=1e-12)
        self.assertEqual(
            rescue_zone_poses(target, yaw, 'normal', GEOMETRY),
            ((0, 1.8, 2.1, 0., -math.pi),),
        )

    def test_cooperative_zones_are_indexed_and_follow_target_axes(self):
        # A valid marker orientation whose target forward axis has yaw pi/2.
        marker_rotation = np.array([
            [-1., 0., 0.],
            [0., 0., 1.],
            [0., 1., 0.],
        ])
        target, yaw = target_planar_pose(
            RigidTransform(marker_rotation, [3., .4, .2]))
        zones = rescue_zone_poses(target, yaw, 'cooperative', GEOMETRY)
        self.assertEqual([zone[0] for zone in zones], [0, 1])
        self.assertAlmostEqual(zones[0][1], 3.3)
        self.assertAlmostEqual(zones[0][2], .9)
        self.assertAlmostEqual(zones[1][1], 2.7)
        self.assertAlmostEqual(zones[1][2], .9)
        self.assertAlmostEqual(zones[0][4], -math.pi / 2)
        self.assertEqual(zones[0][4], zones[1][4])

    def test_nonvertical_marker_and_unknown_target_type_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'marker_not_vertical'):
            target_planar_pose(RigidTransform(np.eye(3), [0., 0., .2]))
        with self.assertRaisesRegex(ValueError, 'target_type'):
            rescue_zone_poses(RigidTransform(np.eye(3), [0., 0., 0.]), 0., 'unknown',
                              GEOMETRY)

    def test_configured_geometry_version_and_shape_are_checked(self):
        validate_development_geometry(GEOMETRY, 'development-v1')
        with self.assertRaisesRegex(ValueError, 'version mismatch'):
            validate_development_geometry(GEOMETRY, 'development-v2')
        invalid = dict(GEOMETRY)
        invalid['cooperative'] = dict(GEOMETRY['cooperative'], lateral_offsets_m=[0.])
        with self.assertRaisesRegex(ValueError, 'two finite offsets'):
            validate_development_geometry(invalid, 'development-v1')


if __name__ == '__main__':
    unittest.main()
