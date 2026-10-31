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

"""Timestamp-aware TF adapter and authoritative multi-robot target registry node."""

from collections import deque
import math
import time

from edgeai_common.config import (
    find_config_dir,
    load_runtime_contract,
    load_yaml,
    resolve_parameters,
)
from edgeai_common.frames import frame, GLOBAL_FRAME, validate_robot_id
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import (
    CompetitionState,
    Target,
    TargetArray,
    TargetObservation,
)
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from .fusion_core import FusionObservation, TargetRegistry, transform_observation
from .geometry import RigidTransform, R_M_T
from .rescue_geometry import rescue_zone_poses, target_planar_pose, validate_development_geometry

ACTIVE_PHASES = (CompetitionState.PHASE_TASK1, CompetitionState.PHASE_TASK2A,
                 CompetitionState.PHASE_TASK2B)
_QUATERNION_EPSILON = 1e-12


def _rotation_from_quaternion(quaternion):
    """Convert a finite nonzero (x, y, z, w) quaternion to a rotation matrix."""
    values = np.asarray(quaternion, dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError('quaternion must contain four finite values')
    norm = float(np.linalg.norm(values))
    if norm < _QUATERNION_EPSILON:
        raise ValueError('quaternion norm must be positive')
    x, y, z, w = values / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def _quaternion_from_rotation(rotation):
    """Convert a right-handed rotation matrix to a normalized (x, y, z, w)."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('rotation must be a finite 3x3 matrix')
    trace = float(np.trace(matrix))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        values = ((matrix[2, 1] - matrix[1, 2]) / scale,
                  (matrix[0, 2] - matrix[2, 0]) / scale,
                  (matrix[1, 0] - matrix[0, 1]) / scale,
                  scale / 4)
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            values = (scale / 4, (matrix[0, 1] + matrix[1, 0]) / scale,
                      (matrix[0, 2] + matrix[2, 0]) / scale,
                      (matrix[2, 1] - matrix[1, 2]) / scale)
        elif index == 1:
            scale = math.sqrt(1 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            values = ((matrix[0, 1] + matrix[1, 0]) / scale, scale / 4,
                      (matrix[1, 2] + matrix[2, 1]) / scale,
                      (matrix[0, 2] - matrix[2, 0]) / scale)
        else:
            scale = math.sqrt(1 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            values = ((matrix[0, 2] + matrix[2, 0]) / scale,
                      (matrix[1, 2] + matrix[2, 1]) / scale, scale / 4,
                      (matrix[1, 0] - matrix[0, 1]) / scale)
    quaternion = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm < _QUATERNION_EPSILON:
        raise ValueError('rotation did not produce a valid quaternion')
    return quaternion / norm


def observation_from_message(message):
    """Convert and validate a ROS observation without changing its timestamp."""
    pose = message.marker_pose.pose
    rotation = _rotation_from_quaternion([
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    ])
    return FusionObservation(
        stamp_ns=message.header.stamp.sec * 1_000_000_000
        + message.header.stamp.nanosec,
        robot_id=message.robot_id,
        marker_id=message.marker_id,
        target_id=message.target_id,
        target_type={
            TargetObservation.TYPE_NORMAL: 'normal',
            TargetObservation.TYPE_COOPERATIVE: 'cooperative',
        }.get(message.target_type, 'unknown'),
        position=np.array([pose.position.x, pose.position.y, pose.position.z]),
        rotation=rotation,
        covariance=np.asarray(message.marker_pose.covariance).reshape(6, 6),
        confidence=message.confidence,
    )


def transform_from_message(message):
    """Convert a geometry_msgs TransformStamped transform to a rigid transform."""
    translation = message.transform.translation
    rotation = message.transform.rotation
    return RigidTransform(
        _rotation_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w]),
        np.array([translation.x, translation.y, translation.z]),
    )


def _planar_pose_covariance(marker_rotation, covariance):
    covariance = np.asarray(covariance, dtype=np.float64)
    if (covariance.shape != (6, 6) or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, atol=1e-8)):
        raise ValueError('marker covariance must be finite and symmetric 6x6')

    target_forward = marker_rotation @ R_M_T[:, 0]
    horizontal_norm_squared = target_forward[0] ** 2 + target_forward[1] ** 2
    if horizontal_norm_squared < 0.5:
        raise ValueError('marker_not_vertical')

    yaw_gradient = np.array([
        -target_forward[1] / horizontal_norm_squared,
        target_forward[0] / horizontal_norm_squared,
        0.,
    ])
    skew_forward = np.array([
        [0., -target_forward[2], target_forward[1]],
        [target_forward[2], 0., -target_forward[0]],
        [-target_forward[1], target_forward[0], 0.],
    ])
    jacobian = np.zeros((6, 6), dtype=np.float64)
    jacobian[:3, :3] = np.eye(3)
    jacobian[5, 3:] = yaw_gradient @ -skew_forward
    result = jacobian @ covariance @ jacobian.T

    # PnP covariance is currently an uncalibrated high-variance placeholder.
    # Keep discarded roll/pitch uncertainty conservative after planar projection.
    orientation_variance = max(
        0., float(np.linalg.eigvalsh(covariance[3:, 3:]).max()))
    result[3, 3] = orientation_variance
    result[4, 4] = orientation_variance
    return (result + result.T) / 2


def target_to_message(estimate, run_id, map_id, map_revision, rescue_geometry=None):
    """Convert a fused estimate to the unchanged shared Target interface."""
    target = Target()
    target.header.frame_id = GLOBAL_FRAME
    target.header.stamp.sec, target.header.stamp.nanosec = divmod(
        estimate.stamp_ns, 1_000_000_000)
    target.run_id = run_id
    target.map_id = map_id
    target.map_revision = map_revision
    target.target_id = estimate.target_id
    target.marker_id = estimate.marker_id
    target.target_type = (Target.TYPE_NORMAL if estimate.target_type == 'normal'
                          else Target.TYPE_COOPERATIVE)
    marker_pose = RigidTransform(estimate.rotation, estimate.position)
    planar_pose, yaw = target_planar_pose(marker_pose)
    target.marker_pose.pose.position.x = float(planar_pose.translation[0])
    target.marker_pose.pose.position.y = float(planar_pose.translation[1])
    target.marker_pose.pose.position.z = float(planar_pose.translation[2])
    quaternion = _quaternion_from_rotation(planar_pose.rotation)
    (target.marker_pose.pose.orientation.x,
     target.marker_pose.pose.orientation.y,
     target.marker_pose.pose.orientation.z,
     target.marker_pose.pose.orientation.w) = map(float, quaternion)
    target.marker_pose.covariance = _planar_pose_covariance(
        estimate.rotation, estimate.covariance).reshape(-1).tolist()
    target.observation_count = estimate.observation_count
    target.confidence = estimate.confidence
    target.observed_by = list(estimate.observed_by)
    target.eligible_for_basic_rescue = False
    if rescue_geometry is not None:
        for index, x, y, z, zone_yaw in rescue_zone_poses(
                planar_pose, yaw, estimate.target_type, rescue_geometry):
            rescue_pose = PoseStamped()
            rescue_pose.header = target.header
            rescue_pose.pose.position.x = x
            rescue_pose.pose.position.y = y
            rescue_pose.pose.position.z = z
            rescue_pose.pose.orientation.z = math.sin(zone_yaw / 2)
            rescue_pose.pose.orientation.w = math.cos(zone_yaw / 2)
            target.rescue_poses.append(rescue_pose)
    return target


class TargetFusion(Node):
    """Fuse known robot-local observations into a versioned map-frame registry."""

    def __init__(self):
        """Load runtime contracts, then create TF, state, and observation endpoints."""
        super().__init__('target_fusion')
        self.declare_parameter('run_mode', 'development')
        self.declare_parameter('config_dir', '')
        config_dir = self.get_parameter('config_dir').value or None
        self.parameters = resolve_parameters(self.get_parameter('run_mode').value, config_dir)
        self.run_mode = self.parameters.run_mode
        self.rescue_geometry = None
        geometry_version = self.parameters.get('rescue.geometry_version')
        if geometry_version is not None:
            if self.run_mode == 'competition':
                raise ValueError(
                    'official rescue geometry is not defined by the current project contract')
            development_config = load_yaml(
                find_config_dir(config_dir) / 'development_scenarios.yaml')
            self.rescue_geometry = development_config.get('rescue_geometry')
            validate_development_geometry(self.rescue_geometry, geometry_version)
        runtime = load_runtime_contract(config_dir)
        timeouts = runtime['timeouts_s']
        self.declare_parameter('robot_ids', 'robot_1,robot_2,robot_3,robot_4')
        self.declare_parameter('tf_lookup_timeout_s', timeouts['tf_lookup'])
        self.declare_parameter(
            'observation_tf_retry_window_s', timeouts['observation_tf_retry_window'])
        self.declare_parameter('observation_stale_s', timeouts['target_observation_stale'])
        self.declare_parameter(
            'competition_state_stale_s', timeouts['competition_state_stale'])
        self.declare_parameter('pending_queue_size', 100)
        robot_ids = [item.strip() for item in self.get_parameter('robot_ids').value.split(',')
                     if item.strip()]
        if not robot_ids or len(set(robot_ids)) != len(robot_ids):
            raise ValueError('robot_ids must be a nonempty comma-separated unique list')
        self.robot_ids = tuple(validate_robot_id(robot_id) for robot_id in robot_ids)
        lookup_timeout = float(self.get_parameter('tf_lookup_timeout_s').value)
        retry_window = float(self.get_parameter('observation_tf_retry_window_s').value)
        stale_seconds = float(self.get_parameter('observation_stale_s').value)
        state_stale = float(self.get_parameter('competition_state_stale_s').value)
        queue_size = int(self.get_parameter('pending_queue_size').value)
        for name, value in (('tf_lookup_timeout_s', lookup_timeout),
                            ('observation_tf_retry_window_s', retry_window),
                            ('observation_stale_s', stale_seconds),
                            ('competition_state_stale_s', state_stale)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if queue_size <= 0:
            raise ValueError('pending_queue_size must be positive')

        perception = self.parameters.require('perception')
        self.registry = TargetRegistry(
            self.parameters.require('aruco.id_mapping'),
            minimum_confidence=perception['minimum_detection_confidence'],
            position_gate_m=perception['fusion_position_gate_m'],
            stale_after_ns=int(stale_seconds * 1e9),
        )
        self.tf_lookup_timeout = Duration(seconds=lookup_timeout)
        self.retry_window_ns = int(retry_window * 1e9)
        self.stale_after_ns = int(stale_seconds * 1e9)
        self.state_stale_ns = int(state_stale * 1e9)
        self.max_pending = queue_size
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sensor_qos = qos_from_contract(runtime, 'sensor_data')
        snapshot_qos = qos_from_contract(runtime, 'snapshot')
        self.observation_subscriptions = [
            self.create_subscription(
                TargetObservation, f'/{robot_id}/target_observations',
                lambda message, expected=robot_id: self._observation_callback(
                    message, expected), self.sensor_qos)
            for robot_id in self.robot_ids
        ]
        self.state = None
        self.state_received_ns = None
        self.competition_context = None
        self.reset_active = False
        self.last_ros_ns = None
        self.pending = deque()
        self.pending_keys = set()
        self.warning_times = {}
        self.publisher = self.create_publisher(
            TargetArray, runtime['topics']['targets'], snapshot_qos)
        self.create_subscription(
            CompetitionState, runtime['topics']['competition_state'],
            self._competition_callback, qos_from_contract(runtime, 'control_state'))
        registry_rate_hz = float(runtime['rates_hz']['target_registry_heartbeat'])
        if not math.isfinite(registry_rate_hz) or registry_rate_hz <= 0:
            raise ValueError('target_registry_heartbeat must be finite and positive')
        period_s = 1.0 / registry_rate_hz
        self.retry_timer = self.create_timer(0.05, self._retry_pending)
        self.publish_timer = self.create_timer(period_s, self._publish_snapshot)
        self.get_logger().info(
            f'target fusion ready; robots={",".join(self.robot_ids)} '
            f'map_frame={GLOBAL_FRAME}')

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _check_clock_jump(self, now_ns):
        if self.last_ros_ns is not None and now_ns < self.last_ros_ns:
            self.registry.reset()
            self.pending.clear()
            self.pending_keys.clear()
            self.state = None
            self.state_received_ns = None
            self.competition_context = None
            self.reset_active = False
            self.get_logger().warning('ROS time moved backwards; cleared fusion state')
        self.last_ros_ns = now_ns

    def _warn_throttled(self, key, message):
        now = time.monotonic()
        if now - self.warning_times.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.warning_times[key] = now

    def _clear_context(self):
        self.registry.reset()
        self.pending.clear()
        self.pending_keys.clear()

    def _competition_callback(self, message):
        now_ns = self._now_ns()
        self._check_clock_jump(now_ns)
        context = (message.run_id, message.map_id, message.map_revision, message.reset_count)
        context_changed = (self.competition_context is not None
                           and context != self.competition_context)
        reset_started = message.reset_in_progress and not self.reset_active
        if context_changed:
            self._clear_context()
            self.get_logger().info('competition run/map/reset changed; cleared target registry')
        if reset_started:
            self._clear_context()
            self.get_logger().info('competition reset began; cleared target registry')
        self.reset_active = message.reset_in_progress
        self.competition_context = context
        self.state = message
        self.state_received_ns = now_ns
        if context_changed or reset_started:
            self._publish_snapshot()

    def _active_state(self, now_ns):
        state = self.state
        return bool(
            state is not None and self.state_received_ns is not None
            and now_ns - self.state_received_ns <= self.state_stale_ns
            and state.run_id and state.map_id
            and state.phase in ACTIVE_PHASES and not state.frozen
            and not state.estop_active and not state.reset_in_progress
            and not self.reset_active)

    def _observation_callback(self, message, expected_robot_id):
        now_ns = self._now_ns()
        self._check_clock_jump(now_ns)
        if not self._active_state(now_ns):
            self._warn_throttled('inactive_state',
                                 'ignoring target observations without fresh active map state')
            return
        if message.run_id != self.state.run_id:
            self._warn_throttled('observation_run',
                                 'ignoring target observation from a different competition run')
            return
        if message.robot_id != expected_robot_id:
            self._warn_throttled(
                'robot_identity',
                f'ignoring observation robot_id={message.robot_id!r} on {expected_robot_id} topic')
            return
        expected_frame = frame(expected_robot_id, 'camera_optical_frame')
        if message.header.frame_id != expected_frame:
            self._warn_throttled(
                'observation_frame',
                f'ignoring {message.header.frame_id!r}; expected {expected_frame!r}')
            return
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if message.header.stamp.sec < 0 or message.header.stamp.nanosec >= 1_000_000_000:
            self._warn_throttled('invalid_stamp', 'ignoring invalid target observation timestamp')
            return
        if stamp_ns > now_ns + 100_000_000:
            self._warn_throttled('future_observation', 'ignoring future-dated target observation')
            return
        if now_ns - stamp_ns > self.stale_after_ns:
            self._warn_throttled('stale_observation', 'ignoring stale target observation')
            return
        target_type = {
            TargetObservation.TYPE_NORMAL: 'normal',
            TargetObservation.TYPE_COOPERATIVE: 'cooperative',
        }.get(message.target_type)
        if self.registry.definitions.get(message.marker_id) != (
                message.target_id, target_type):
            self._warn_throttled(
                'target_identity',
                'ignoring unconfigured marker/target identity '
                f'{message.marker_id}/{message.target_id}')
            return
        key = (message.robot_id, message.target_id, stamp_ns)
        if key in self.pending_keys:
            return
        if len(self.pending) >= self.max_pending:
            self._warn_throttled('pending_full', 'dropping observation: TF retry queue is full')
            return
        expires_ns = now_ns + self.retry_window_ns
        if not self._try_transform(message):
            self.pending.append((message, key, expires_ns))
            self.pending_keys.add(key)

    def _try_transform(self, message):
        now_ns = self._now_ns()
        if not self._active_state(now_ns):
            return False
        stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        try:
            transform_message = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME,
                message.header.frame_id,
                Time(nanoseconds=stamp_ns, clock_type=self.get_clock().clock_type),
                timeout=self.tf_lookup_timeout,
            )
            observation = observation_from_message(message)
            mapped = transform_observation(
                observation, transform_from_message(transform_message))
            target_planar_pose(RigidTransform(mapped.rotation, mapped.position))
        except TransformException:
            return False
        except (ValueError, TypeError, np.linalg.LinAlgError) as error:
            reason = str(error)
            self._warn_throttled(
                'invalid_geometry' if reason == 'marker_not_vertical' else 'invalid_observation',
                f'ignoring invalid target observation: {reason}')
            return True
        if self.registry.ingest(mapped, now_ns):
            self._publish_snapshot()
        else:
            self._warn_throttled(
                'rejected_observation',
                'target observation rejected by identity, freshness, confidence, '
                'duplicate, or spatial-consistency checks')
        return True

    def _retry_pending(self):
        now_ns = self._now_ns()
        self._check_clock_jump(now_ns)
        if not self._active_state(now_ns):
            if self.pending:
                self.pending.clear()
                self.pending_keys.clear()
            if self.registry.prune(now_ns):
                self._publish_snapshot()
            return
        pending = list(self.pending)
        self.pending.clear()
        self.pending_keys.clear()
        for message, key, expires_ns in pending:
            stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            if now_ns - stamp_ns > self.stale_after_ns or now_ns > expires_ns:
                self._warn_throttled('tf_timeout',
                                     'dropping observation after timestamped TF retry window')
                continue
            if not self._try_transform(message):
                self.pending.append((message, key, expires_ns))
                self.pending_keys.add(key)
        if self.registry.prune(now_ns):
            self._publish_snapshot()

    def _publish_snapshot(self):
        now_ns = self._now_ns()
        self._check_clock_jump(now_ns)
        state = self.state
        if state is None or not state.run_id:
            return
        self.registry.prune(now_ns)
        array = TargetArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = GLOBAL_FRAME
        array.run_id = state.run_id
        array.map_id = state.map_id
        array.map_revision = state.map_revision
        array.revision = self.registry.revision
        array.task1_frozen = False
        array.targets = [
            target_to_message(estimate, state.run_id, state.map_id, state.map_revision)
            if self.rescue_geometry is None
            else target_to_message(
                estimate, state.run_id, state.map_id, state.map_revision,
                self.rescue_geometry)
            for estimate in self.registry.snapshot()
        ]
        self.publisher.publish(array)


def main(args=None):
    """Run the fusion node with TF listener callbacks serviced during bounded lookups."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = TargetFusion()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
