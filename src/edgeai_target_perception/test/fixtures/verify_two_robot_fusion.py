#!/usr/bin/env python3
"""Verify the two-robot Gazebo fusion run and its stale-observation gate."""

import argparse
import collections
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'src' / 'edgeai_common'))

from edgeai_common.config import load_runtime_contract  # noqa: E402
from edgeai_common.qos import qos_from_contract  # noqa: E402
from edgeai_robot_interfaces.msg import TargetArray, TargetObservation  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

ROBOT_IDS = ('robot_1', 'robot_2')
RUN_MAP_ID = 'test-two-robot-ground-truth'
MARKER_POSITION = np.array([-0.45, -1.15, 0.15])


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class FusionCheck(Node):
    """Collect observations and fused snapshots or inject one deliberately stale sample."""

    def __init__(self, run_id):
        """Create sensor, observation, and target subscriptions for one test run."""
        super().__init__(
            'two_robot_fusion_check',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.run_id = run_id
        runtime = load_runtime_contract()
        self.observations = {}
        self.observation_stamps = collections.defaultdict(set)
        self.image_stamps = collections.defaultdict(set)
        self.images = {}
        self.depth_images = {}
        self.camera_infos = {}
        self.last_observation_at = time.monotonic()
        self.injected_stale_stamp_ns = None
        self.latest_array = None
        self.observation_errors = []
        sensor_qos = qos_from_contract(runtime, 'sensor_data')
        self.stream_subscriptions = [
            self.create_subscription(
                TargetObservation, f'/{robot_id}/target_observations',
                lambda message, expected=robot_id: self._on_observation(message, expected),
                sensor_qos,
            )
            for robot_id in ROBOT_IDS
        ]
        for robot_id in ROBOT_IDS:
            self.stream_subscriptions.extend([
                self.create_subscription(
                    Image, f'/{robot_id}/camera/image',
                    lambda message, expected=robot_id: self._on_image(
                        message, expected, self.images),
                    sensor_qos,
                ),
                self.create_subscription(
                    Image, f'/{robot_id}/camera/depth_image',
                    lambda message, expected=robot_id: self._on_image(
                        message, expected, self.depth_images),
                    sensor_qos,
                ),
                self.create_subscription(
                    CameraInfo, f'/{robot_id}/camera/camera_info',
                    lambda message, expected=robot_id: self._on_camera_info(message, expected),
                    sensor_qos,
                ),
            ])
        self.array_subscription = self.create_subscription(
            TargetArray, '/edge/targets', self._on_array,
            qos_from_contract(runtime, 'snapshot'),
        )
        self.stale_publisher = self.create_publisher(
            TargetObservation, '/robot_1/target_observations',
            sensor_qos,
        )

    def _on_observation(self, message, expected_robot_id):
        stamp_ns = _stamp_ns(message.header.stamp)
        if stamp_ns == self.injected_stale_stamp_ns:
            return
        self.last_observation_at = time.monotonic()
        self.observations[expected_robot_id] = message
        self.observation_stamps[expected_robot_id].add(stamp_ns)
        if (message.robot_id != expected_robot_id
                or message.header.frame_id != f'{expected_robot_id}/camera_optical_frame'
                or message.run_id != self.run_id
                or message.marker_id != 1
                or message.target_id != 'T1'
                or stamp_ns <= 0
                or not math.isfinite(message.confidence)
                or message.confidence <= 0):
            self.observation_errors.append(
                f'invalid observation on /{expected_robot_id}/target_observations')

    def _on_image(self, message, expected_robot_id, collection):
        collection[expected_robot_id] = message
        if collection is self.images:
            self.image_stamps[expected_robot_id].add(_stamp_ns(message.header.stamp))

    def _on_camera_info(self, message, expected_robot_id):
        self.camera_infos[expected_robot_id] = message

    def _on_array(self, message):
        self.latest_array = message

    def _spin_until(self, predicate, timeout_s, failure):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.observation_errors:
                raise RuntimeError('; '.join(self.observation_errors))
            if predicate():
                return
        raise TimeoutError(failure)

    def verify_fusion(self, timeout_s):
        """Require both valid camera observations to contribute to one T1 target."""
        def has_two_robot_target():
            if self.latest_array is None:
                return False
            targets = self.latest_array.targets
            return (
                len(self.observations) == len(ROBOT_IDS)
                and all(
                    _stamp_ns(self.observations[robot_id].header.stamp)
                    in self.image_stamps[robot_id]
                    for robot_id in ROBOT_IDS
                )
                and all(
                    robot_id in self.images
                    and robot_id in self.depth_images
                    and robot_id in self.camera_infos
                    for robot_id in ROBOT_IDS
                )
                and len(targets) == 1
                and targets[0].target_id == 'T1'
                and tuple(targets[0].observed_by) == ROBOT_IDS
                and targets[0].observation_count == 2
            )

        self._spin_until(
            has_two_robot_target, timeout_s,
            'did not receive one T1 fused from both robots',
        )
        array = self.latest_array
        target, = array.targets
        if (array.header.frame_id != 'map' or array.run_id != self.run_id
                or array.map_id != RUN_MAP_ID or array.map_revision != 1):
            raise AssertionError('TargetArray run/map identity or frame is incorrect')
        for robot_id in ROBOT_IDS:
            image = self.images.get(robot_id)
            depth = self.depth_images.get(robot_id)
            camera_info = self.camera_infos.get(robot_id)
            expected_frame = f'{robot_id}/camera_optical_frame'
            if any(item is None for item in (image, depth, camera_info)):
                raise AssertionError(f'{robot_id} RGB-D or CameraInfo stream is missing')
            if any(item.header.frame_id != expected_frame
                   for item in (image, depth, camera_info)):
                raise AssertionError(f'{robot_id} camera streams have an invalid optical frame')
            if (image.width != 640 or image.height != 480
                    or depth.width != image.width or depth.height != image.height
                    or depth.encoding != '32FC1'
                    or camera_info.width != image.width or camera_info.height != image.height
                    or camera_info.k[0] <= 0 or camera_info.k[4] <= 0):
                raise AssertionError(f'{robot_id} RGB-D calibration or image shape is invalid')
            if abs(_stamp_ns(image.header.stamp) - _stamp_ns(depth.header.stamp)) > 50_000_000:
                raise AssertionError(f'{robot_id} RGB and depth timestamps are misaligned')
            camera_info_delta = abs(
                _stamp_ns(image.header.stamp) - _stamp_ns(camera_info.header.stamp))
            if camera_info_delta > 50_000_000:
                raise AssertionError(f'{robot_id} RGB and CameraInfo timestamps are misaligned')
            if _stamp_ns(self.observations[robot_id].header.stamp) not in self.image_stamps[
                    robot_id]:
                raise AssertionError(f'{robot_id} observation timestamp differs from RGB input')
        if array.revision < 2:
            raise AssertionError(f'registry revision did not record both robots: {array.revision}')
        target_stamp = _stamp_ns(target.header.stamp)
        all_observation_stamps = set().union(*self.observation_stamps.values())
        if target_stamp not in all_observation_stamps:
            raise AssertionError('fused target timestamp is not an input observation timestamp')
        position = np.array([
            target.marker_pose.pose.position.x,
            target.marker_pose.pose.position.y,
            target.marker_pose.pose.position.z,
        ])
        if not np.isfinite(position).all():
            raise AssertionError('fused target position is not finite')
        error = float(np.linalg.norm(position - MARKER_POSITION))
        if error > 0.15:
            raise AssertionError(
                f'fused position differs from test marker ground truth by {error:.3f} m')
        print(
            f'PASS observations: robot_1={_stamp_ns(self.observations["robot_1"].header.stamp)} '
            f'robot_2={_stamp_ns(self.observations["robot_2"].header.stamp)} '
            f'run_id={self.run_id}'
        )
        print(
            f'PASS fusion: count={target.observation_count} '
            f'observed_by={list(target.observed_by)} '
            f'revision={array.revision} target_stamp_ns={target_stamp} '
            f'position={position.tolist()} ground_truth_error_m={error:.4f}'
        )

    def verify_stale_rejection(self, timeout_s):
        """Prove stale input cannot recreate a target or advance the registry revision."""
        quiet_s = 2.5

        def registry_is_empty_and_quiet():
            return (
                self.latest_array is not None
                and not self.latest_array.targets
                and time.monotonic() - self.last_observation_at >= quiet_s
            )

        self._spin_until(
            registry_is_empty_and_quiet, timeout_s,
            'detectors must be stopped and registry must expire before stale check',
        )
        revision = self.latest_array.revision
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 5_000_000_000:
            raise RuntimeError('simulation clock has not advanced enough for stale injection')
        stale = TargetObservation()
        stale.header.stamp.sec, stale.header.stamp.nanosec = divmod(
            now_ns - 5_000_000_000, 1_000_000_000)
        self.injected_stale_stamp_ns = _stamp_ns(stale.header.stamp)
        stale.header.frame_id = 'robot_1/camera_optical_frame'
        stale.run_id = self.run_id
        stale.robot_id = 'robot_1'
        stale.marker_id = 1
        stale.target_id = 'T1'
        stale.target_type = TargetObservation.TYPE_NORMAL
        stale.marker_pose.pose.orientation.w = 1.0
        stale.confidence = 0.9
        for index in (0, 7, 14, 21, 28, 35):
            stale.marker_pose.covariance[index] = 0.02
        for _ in range(3):
            self.stale_publisher.publish(stale)
            rclpy.spin_once(self, timeout_sec=0.1)
        self._spin_until(
            lambda: self.latest_array is not None
            and self.latest_array.revision >= revision
            and not self.latest_array.targets
            and time.monotonic() - self.last_observation_at >= quiet_s,
            2.0,
            'stale observation recreated a target or a detector is still active',
        )
        if self.latest_array.revision != revision:
            raise AssertionError(
                f'stale observation changed registry revision {revision} '
                f'-> {self.latest_array.revision}')
        print(f'PASS stale rejection: target_count=0 revision_unchanged={revision}')


def main():
    """Run the requested two-robot fusion integration check."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('verify', 'stale'), default='verify')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--timeout-s', type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error('--timeout-s must be finite and positive')

    rclpy.init()
    node = None
    try:
        node = FusionCheck(args.run_id)
        if args.mode == 'verify':
            node.verify_fusion(args.timeout_s)
        else:
            node.verify_stale_rejection(args.timeout_s)
    except (AssertionError, RuntimeError, TimeoutError) as error:
        print(f'FAIL {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
