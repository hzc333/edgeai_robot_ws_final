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

"""ROS adapter tests for timestamped TF lookup, state resets, and TargetArray output."""

from pathlib import Path
import math
import sys
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[1]
sys.path[:0] = [str(PACKAGE), str(ROOT / 'src' / 'edgeai_common')]

from edgeai_robot_interfaces.msg import CompetitionState, TargetObservation  # noqa: E402
from edgeai_target_perception.fusion_core import (  # noqa: E402
    FusionObservation,
    TargetRegistry,
    transform_observation,
)
from edgeai_target_perception.geometry import RigidTransform  # noqa: E402
from edgeai_target_perception.target_fusion_node import (  # noqa: E402
    TargetFusion,
    target_to_message,
)
from geometry_msgs.msg import TransformStamped  # noqa: E402
import rclpy  # noqa: E402
from tf2_ros import LookupException  # noqa: E402


class FakeBuffer:
    def __init__(self):
        self.fail = False
        self.requested = []

    def lookup_transform(self, target_frame, source_frame, stamp, timeout):
        self.requested.append((target_frame, source_frame, stamp.nanoseconds, timeout))
        if self.fail:
            raise LookupException('transform not available yet')
        result = TransformStamped()
        result.header.frame_id = target_frame
        result.child_frame_id = source_frame
        result.transform.rotation.w = 1.0
        return result


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _state(run_id='run-a', map_id='map-a', map_revision=1, reset_count=0):
    state = CompetitionState()
    state.run_id = run_id
    state.map_id = map_id
    state.map_revision = map_revision
    state.reset_count = reset_count
    state.phase = CompetitionState.PHASE_TASK1
    return state


def _observation(stamp_ns, robot_id='robot_1', marker_id=1, target_id='T1',
                 run_id='run-a'):
    message = TargetObservation()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.header.frame_id = f'{robot_id}/camera_optical_frame'
    message.run_id = run_id
    message.robot_id = robot_id
    message.marker_id = marker_id
    message.target_id = target_id
    message.target_type = TargetObservation.TYPE_NORMAL
    message.marker_pose.pose.position.x = .2
    message.marker_pose.pose.position.y = -.1
    message.marker_pose.pose.position.z = 1.0
    message.marker_pose.pose.orientation.x = 0.5
    message.marker_pose.pose.orientation.y = 0.5
    message.marker_pose.pose.orientation.z = 0.5
    message.marker_pose.pose.orientation.w = 0.5
    for index in (0, 7, 14, 21, 28, 35):
        message.marker_pose.covariance[index] = .02
    message.confidence = .9
    return message


class TargetFusionNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.node = TargetFusion()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = type(self).node
        self.node.registry.reset()
        self.node.pending.clear()
        self.node.pending_keys.clear()
        self.node.state = None
        self.node.state_received_ns = None
        self.node.competition_context = None
        self.node.reset_active = False
        self.buffer = FakeBuffer()
        self.node.tf_buffer = self.buffer
        self.publisher = FakePublisher()
        self.node.publisher = self.publisher
        self.node._competition_callback(_state())

    def test_timestamped_lookup_and_map_target_publication(self):
        stamp_ns = self.node.get_clock().now().nanoseconds
        self.node._observation_callback(_observation(stamp_ns), 'robot_1')
        self.assertEqual(len(self.buffer.requested), 1)
        target_frame, source_frame, requested_ns, _ = self.buffer.requested[0]
        self.assertEqual(target_frame, 'map')
        self.assertEqual(source_frame, 'robot_1/camera_optical_frame')
        self.assertEqual(requested_ns, stamp_ns)
        self.assertEqual(len(self.publisher.messages), 1)
        array = self.publisher.messages[0]
        self.assertEqual(array.header.frame_id, 'map')
        self.assertEqual(array.run_id, 'run-a')
        self.assertEqual(array.map_id, 'map-a')
        self.assertEqual(array.map_revision, 1)
        self.assertEqual(array.revision, 1)
        self.assertFalse(array.task1_frozen)
        target, = array.targets
        self.assertEqual(target.target_id, 'T1')
        self.assertEqual(target.marker_id, 1)
        self.assertEqual(target.observed_by, ['robot_1'])
        self.assertEqual(target.observation_count, 1)
        self.assertFalse(target.eligible_for_basic_rescue)
        self.assertEqual(len(target.rescue_poses), 1)
        self.assertAlmostEqual(target.rescue_poses[0].pose.position.x, .7)
        self.assertAlmostEqual(target.rescue_poses[0].pose.position.y, -.1)
        self.assertAlmostEqual(target.rescue_poses[0].pose.position.z, 0.)

    def test_target_covariance_matches_map_frame_planar_pose(self):
        marker_rotation = np.array([
            [0., 0., 1.],
            [0., 1., 0.],
            [-1., 0., 0.],
        ])
        map_rotation = np.array([
            [0., -1., 0.],
            [1., 0., 0.],
            [0., 0., 1.],
        ])
        source = FusionObservation(
            stamp_ns=1_000_000_000,
            robot_id='robot_1',
            marker_id=1,
            target_id='T1',
            target_type='normal',
            position=np.array([1., 2., 3.]),
            rotation=marker_rotation,
            covariance=np.diag([1., 2., 3., 4., 5., 6.]),
            confidence=.9,
        )
        mapped = transform_observation(
            source, RigidTransform(map_rotation, np.zeros(3)))
        registry = TargetRegistry({1: {'target_id': 'T1', 'type': 'normal'}})
        self.assertTrue(registry.ingest(mapped, mapped.stamp_ns))
        estimate, = registry.snapshot()

        target = target_to_message(estimate, 'run-a', 'map-a', 1)
        covariance = np.asarray(target.marker_pose.covariance).reshape(6, 6)
        self.assertAlmostEqual(
            math.atan2(
                2 * target.marker_pose.pose.orientation.w
                * target.marker_pose.pose.orientation.z,
                1 - 2 * target.marker_pose.pose.orientation.z ** 2,
            ),
            math.pi / 2,
        )
        np.testing.assert_allclose(
            np.diag(covariance), [2., 1., 3., 6., 6., 6.], atol=1e-12)
        np.testing.assert_allclose(covariance, covariance.T, atol=1e-12)
        self.assertTrue(np.isfinite(covariance).all())

    def test_nonvertical_marker_is_rejected_before_registry_update(self):
        message = _observation(self.node.get_clock().now().nanoseconds)
        message.marker_pose.pose.orientation.x = 0.
        message.marker_pose.pose.orientation.y = 0.
        message.marker_pose.pose.orientation.z = 0.
        message.marker_pose.pose.orientation.w = 1.
        self.node._observation_callback(message, 'robot_1')
        self.assertEqual(self.node.registry.snapshot(), ())
        self.assertEqual(self.node.registry.revision, 0)

    def test_missing_timestamped_tf_is_retried_then_fused(self):
        self.buffer.fail = True
        stamp_ns = self.node.get_clock().now().nanoseconds
        self.node._observation_callback(_observation(stamp_ns), 'robot_1')
        self.assertEqual(len(self.node.pending), 1)
        self.buffer.fail = False
        self.node._retry_pending()
        self.assertEqual(len(self.node.pending), 0)
        self.assertEqual(self.node.registry.snapshot()[0].target_id, 'T1')
        self.assertEqual(len(self.publisher.messages), 1)

    def test_run_map_revision_and_reset_clear_registry(self):
        stamp_ns = self.node.get_clock().now().nanoseconds
        self.node._observation_callback(_observation(stamp_ns), 'robot_1')
        self.assertEqual(len(self.node.registry.snapshot()), 1)
        self.node._competition_callback(
            _state(run_id='run-b', map_id='map-b', map_revision=2, reset_count=1))
        self.assertEqual(self.node.registry.snapshot(), ())
        self.assertEqual(self.node.registry.revision, 0)
        self.assertEqual(self.publisher.messages[-1].run_id, 'run-b')
        self.assertEqual(self.publisher.messages[-1].targets, [])
        state = _state(run_id='run-b', map_id='map-b', map_revision=2, reset_count=1)
        state.reset_in_progress = True
        self.node._competition_callback(state)
        self.node._observation_callback(
            _observation(self.node.get_clock().now().nanoseconds), 'robot_1')
        self.assertEqual(self.node.registry.snapshot(), ())

    def test_stale_and_cross_robot_spoofed_messages_are_ignored(self):
        now_ns = self.node.get_clock().now().nanoseconds
        self.node._observation_callback(
            _observation(now_ns - 3_000_000_000), 'robot_1')
        self.node._observation_callback(
            _observation(now_ns, robot_id='robot_2'), 'robot_1')
        self.assertEqual(self.buffer.requested, [])
        self.assertEqual(self.node.registry.snapshot(), ())

    def test_stale_observation_does_not_change_fused_registry_revision(self):
        now_ns = self.node.get_clock().now().nanoseconds
        self.node._observation_callback(_observation(now_ns), 'robot_1')
        revision = self.node.registry.revision
        self.node._observation_callback(
            _observation(now_ns - 3_000_000_000, robot_id='robot_2'), 'robot_2')
        self.assertEqual(self.node.registry.revision, revision)
        self.assertEqual(self.node.registry.snapshot()[0].observed_by, ('robot_1',))

    def test_observation_from_previous_run_is_rejected(self):
        self.node._competition_callback(_state(run_id='run-b'))
        self.node._observation_callback(
            _observation(self.node.get_clock().now().nanoseconds), 'robot_1')
        self.assertEqual(self.buffer.requested, [])
        self.assertEqual(self.node.registry.snapshot(), ())

    def test_stale_competition_state_prevents_tf_lookup(self):
        self.node.state_received_ns -= self.node.state_stale_ns + 1
        self.node._observation_callback(
            _observation(self.node.get_clock().now().nanoseconds), 'robot_1')
        self.assertEqual(self.buffer.requested, [])

    def test_simulated_clock_rewind_clears_state_and_pending_evidence(self):
        stamp_ns = self.node.get_clock().now().nanoseconds
        self.node._observation_callback(_observation(stamp_ns), 'robot_1')
        previous_clock = self.node.last_ros_ns
        self.node._check_clock_jump(previous_clock - 1)
        self.assertIsNone(self.node.state)
        self.assertIsNone(self.node.competition_context)
        self.assertEqual(self.node.registry.snapshot(), ())


if __name__ == '__main__':
    unittest.main()
