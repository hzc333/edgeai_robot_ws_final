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

"""ROS 2 adapter that publishes robot-local ArUco target observations."""

import math
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from edgeai_common.config import load_runtime_contract, resolve_parameters
from edgeai_common.frames import frame, topic, validate_robot_id
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState, TargetObservation
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .aruco_core import ArucoCore, MarkerResult
from .geometry import depth_to_meters, validate_camera_calibration

DETECTOR_VERSION = 'aruco-ippe-square-phase3-v1'
DEPTH_IMAGE_SCALES_M = {'16UC1': 0.001, '32FC1': 1.0}
ACTIVE_PHASES = (CompetitionState.PHASE_TASK1, CompetitionState.PHASE_TASK2A,
                 CompetitionState.PHASE_TASK2B)


def _quaternion_from_rotation(rotation):
    """Convert a right-handed 3x3 rotation matrix into an (x, y, z, w) quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('rotation must be a finite 3x3 matrix')
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        quaternion = ((matrix[2, 1] - matrix[1, 2]) / scale,
                      (matrix[0, 2] - matrix[2, 0]) / scale,
                      (matrix[1, 0] - matrix[0, 1]) / scale,
                      0.25 * scale)
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            quaternion = (0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale,
                          (matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[2, 1] - matrix[1, 2]) / scale)
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            quaternion = ((matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale,
                          (matrix[0, 2] - matrix[2, 0]) / scale)
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            quaternion = ((matrix[0, 2] + matrix[2, 0]) / scale,
                          (matrix[1, 2] + matrix[2, 1]) / scale, 0.25 * scale,
                          (matrix[1, 0] - matrix[0, 1]) / scale)
    values = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(values))
    if not np.isfinite(norm) or norm <= 0:
        raise ValueError('rotation did not produce a valid quaternion')
    return values / norm


def observations_from_results(results, header, robot_id, run_id):
    """Convert only known, valid and non-ambiguous results into interface messages."""
    validate_robot_id(robot_id)
    if not isinstance(run_id, str) or not run_id:
        raise ValueError('run_id must be a nonempty string')
    observations = []
    for result in results:
        if (not isinstance(result, MarkerResult) or not result.valid
                or result.identity is None or result.ambiguity is None
                or result.ambiguity.ambiguous):
            continue
        if result.identity.target_type == 'normal':
            target_type = TargetObservation.TYPE_NORMAL
        elif result.identity.target_type == 'cooperative':
            target_type = TargetObservation.TYPE_COOPERATIVE
        else:
            continue
        if result.tvec is None or result.rvec is None or result.reprojection_error_px is None:
            continue
        rotation = cv2.Rodrigues(np.asarray(result.rvec, dtype=np.float64).reshape(3, 1))[0]
        translation = np.asarray(result.tvec, dtype=np.float64).reshape(3)
        if not np.isfinite(translation).all() or not np.isfinite(result.reprojection_error_px):
            continue
        quaternion = _quaternion_from_rotation(rotation)
        observation = TargetObservation()
        observation.header = header
        observation.run_id = run_id
        observation.robot_id = robot_id
        observation.marker_id = result.marker_id
        observation.target_id = result.identity.target_id
        observation.target_type = target_type
        observation.marker_pose.pose.position.x = float(translation[0])
        observation.marker_pose.pose.position.y = float(translation[1])
        observation.marker_pose.pose.position.z = float(translation[2])
        (observation.marker_pose.pose.orientation.x,
         observation.marker_pose.pose.orientation.y,
         observation.marker_pose.pose.orientation.z,
         observation.marker_pose.pose.orientation.w) = map(float, quaternion)
        # No calibrated covariance model exists yet; represent unknown uncertainty
        # conservatively rather than implying a zero-variance estimate.
        for index in (0, 7, 14, 21, 28, 35):
            observation.marker_pose.covariance[index] = 1e6
        observation.confidence = float(math.exp(-0.5 * result.reprojection_error_px ** 2))
        observation.detector_version = DETECTOR_VERSION
        observations.append(observation)
    return observations


def detect_observations(core, image, camera_matrix, distortion, header, robot_id, run_id,
                        depth_image=None):
    """Run the geometry core and convert its accepted results to ROS observations."""
    results = core.detect(
        image, camera_matrix, distortion, depth_image=depth_image,
        depth_scale_m=1.0 if depth_image is not None else None)
    return results, observations_from_results(results, header, robot_id, run_id)


class TargetDetector(Node):
    """Detect configured ArUco markers in one robot's calibrated camera stream."""

    def __init__(self):
        super().__init__('target_detector')
        self.declare_parameter('robot_id', 'robot_1')
        self.declare_parameter('run_mode', 'development')
        self.declare_parameter('config_dir', '')
        self.declare_parameter('use_depth', False)
        self.declare_parameter('depth_sync_tolerance_s', 0.05)
        self.declare_parameter('camera_image_topic', 'camera/image')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('camera_depth_topic', 'camera/depth_image')
        self.declare_parameter('observation_topic', 'target_observations')
        self.robot_id = validate_robot_id(self.get_parameter('robot_id').value)
        run_mode = self.get_parameter('run_mode').value
        config_dir = self.get_parameter('config_dir').value or None
        self.parameters = resolve_parameters(run_mode, config_dir)
        self.core = ArucoCore.from_resolved_parameters(self.parameters)
        self.camera_frame = frame(self.robot_id, 'camera_optical_frame')
        self.depth_enabled = bool(self.get_parameter('use_depth').value)
        self.depth_sync_tolerance_s = float(
            self.get_parameter('depth_sync_tolerance_s').value)
        if not math.isfinite(self.depth_sync_tolerance_s) or self.depth_sync_tolerance_s <= 0:
            raise ValueError('depth_sync_tolerance_s must be finite and positive')
        contract = load_runtime_contract(config_dir)
        sensor_qos = qos_from_contract(contract, 'sensor_data')
        state_qos = qos_from_contract(contract, 'control_state')
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_message = None
        self.competition = None
        self.competition_received_at = None
        self.warning_times = {}
        self.observation_pub = self.create_publisher(
            TargetObservation,
            topic(self.robot_id, self.get_parameter('observation_topic').value), sensor_qos)
        self.create_subscription(
            CompetitionState, '/competition/state', self._competition_callback, state_qos)
        self.create_subscription(
            CameraInfo, topic(self.robot_id, self.get_parameter('camera_info_topic').value),
            self._camera_info_callback, sensor_qos)
        self.create_subscription(
            Image, topic(self.robot_id, self.get_parameter('camera_image_topic').value),
            self._image_callback, sensor_qos)
        if self.depth_enabled:
            self.create_subscription(
                Image, topic(self.robot_id, self.get_parameter('camera_depth_topic').value),
                self._depth_callback, sensor_qos)
        self.get_logger().info(
            f'target detector ready for {self.robot_id}; depth_enabled={self.depth_enabled}')

    def _warn_throttled(self, key, message, interval_s=5.0):
        now = time.monotonic()
        if now - self.warning_times.get(key, -math.inf) >= interval_s:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def _competition_callback(self, message):
        self.competition = message
        self.competition_received_at = time.monotonic()

    def _camera_info_callback(self, message):
        if message.header.frame_id != self.camera_frame:
            self._warn_throttled(
                'camera_frame',
                f'ignoring CameraInfo in {message.header.frame_id!r}; '
                f'expected {self.camera_frame!r}')
            self.camera_info = None
            return
        try:
            validate_camera_calibration(np.asarray(message.k).reshape(3, 3), message.d)
        except ValueError as error:
            self._warn_throttled('calibration', f'invalid CameraInfo: {error}')
            self.camera_info = None
            return
        self.camera_info = message

    def _depth_callback(self, message):
        self.depth_message = message

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _current_run_id(self):
        state = self.competition
        if (state is None or not state.run_id
                or self.competition_received_at is None
                or time.monotonic() - self.competition_received_at > 3.0
                or state.phase not in ACTIVE_PHASES or state.frozen or state.estop_active):
            return None
        return state.run_id

    def _aligned_depth(self, image_message, image_shape):
        message = self.depth_message
        if message is None:
            self._warn_throttled(
                'missing_depth', 'no aligned depth image received; using PnP only')
            return None
        if message.header.frame_id != self.camera_frame:
            raise ValueError('depth image frame does not match the camera optical frame')
        if abs(self._stamp_seconds(message.header.stamp)
               - self._stamp_seconds(image_message.header.stamp)) > self.depth_sync_tolerance_s:
            self._warn_throttled('stale_depth', 'depth image is outside the RGB sync tolerance')
            return None
        depth = self.bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')
        scale = DEPTH_IMAGE_SCALES_M.get(message.encoding)
        if scale is None:
            raise ValueError(f'unsupported depth encoding {message.encoding!r}')
        depth_m = depth_to_meters(depth, unit_scale_m=scale)
        if depth_m.shape != image_shape:
            raise ValueError('depth image dimensions do not match the RGB image')
        return depth_m

    def _image_callback(self, message):
        if message.header.frame_id != self.camera_frame:
            self._warn_throttled(
                'image_frame',
                f'ignoring image in {message.header.frame_id!r}; expected {self.camera_frame!r}')
            return
        run_id = self._current_run_id()
        if run_id is None:
            self._warn_throttled('run_state', 'waiting for an active competition run state')
            return
        if self.camera_info is None:
            self._warn_throttled('camera_info', 'waiting for valid CameraInfo')
            return
        if (self.camera_info.width != message.width
                or self.camera_info.height != message.height):
            self._warn_throttled('image_size', 'image dimensions do not match CameraInfo')
            return
        try:
            matrix, coefficients = validate_camera_calibration(
                np.asarray(self.camera_info.k).reshape(3, 3), self.camera_info.d)
            if message.encoding not in ('rgb8', 'bgr8', 'mono8'):
                raise ValueError(f'unsupported RGB image encoding {message.encoding!r}')
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            depth = self._aligned_depth(message, image.shape[:2]) if self.depth_enabled else None
            results, observations = detect_observations(
                self.core, image, matrix, coefficients, message.header,
                self.robot_id, run_id, depth)
        except (ValueError, CvBridgeError, cv2.error) as error:
            self._warn_throttled('frame_processing', f'rejecting camera frame: {error}')
            return
        rejections = {}
        for result in results:
            if result.rejection_reason:
                rejections[result.rejection_reason] = (
                    rejections.get(result.rejection_reason, 0) + 1)
        rejected_ambiguous = sum(
            result.valid and result.ambiguity is not None and result.ambiguity.ambiguous
            for result in results)
        for reason, count in rejections.items():
            self._warn_throttled(
                f'rejected_{reason}', f'rejected {count} marker result(s): {reason}')
        if rejected_ambiguous:
            self._warn_throttled(
                'ambiguous_pose', 'withholding planar PnP pose due to ambiguity')
        for observation in observations:
            self.observation_pub.publish(observation)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TargetDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
