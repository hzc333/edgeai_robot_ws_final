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

"""
Localization wrapper of one robot: the /<robot_id>/relocalize service (P1 plan 3-02).

It owns the AMCL lifecycle of its robot and, after the first successful
relocalization, activates the robot's Nav2 navigation nodes (Nav2 cannot
activate before map -> base_footprint exists). Modes:

* MODE_KNOWN_START_POSE: a pose from the resolved localization.start_poses,
  which are given in arena coordinates; the map frame must coincide with the
  arena frame (P2 initialises SLAM accordingly).
* MODE_POSE_HINT: the pose and covariance in the request (map frame). Whether
  an operator may provide a hint is checked by the caller (system_manager).
  A hint without covariance gets default_hint_*_std. The one exception is the
  SLAM -> AMCL handover of the robot that ran SLAM (slam_handover, set by the
  launch file in slam mode): its first relocalization during HANDOVER gets the
  tighter handover_hint_*_std. That hint is the SLAM pose on the map being
  frozen; a wide particle cloud only lets AMCL settle on its own grid-quantised
  optimum a few centimetres away (a pose jump). Placement hints for the other
  robots can be off by 0.1 m or more and keep the wide default, otherwise a tight
  cloud would converge confidently next to the true pose.
* MODE_GLOBAL: AMCL global re-initialisation followed by no-motion updates;
  unreliable in a near-symmetric arena and reported as such.

Success requires the AMCL x/y and yaw standard deviations to stay below the
convergence limits for convergence_hold_s AND the current laser scan, projected
with the estimated pose, to agree with the map (scan_match_min_ratio of the
valid beam endpoints near an occupied cell). AMCL converges confidently around
a wrong hint, so the covariance alone cannot detect a wrong pose.

Parking (parked_start_pose_id): a robot that waits at a known start pose while
another robot maps (midterm Part 1, ADR-0001) has no map pose, so the other
robots cannot see its chassis (it is below their LiDAR plane). Until its first
relocalization this node then publishes map -> robot_N/odom from that start
pose; it stops before starting AMCL, so the transform has one publisher at a time.
SLAM Toolbox anchors its map at the mapping robot's start pose (it ignores
map_start_pose while mapping); parked_map_origin_pose_id names that pose, so the
parked pose is expressed in the SLAM map. Leave it empty when map is the arena frame.
"""

import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from edgeai_common.config import load_runtime_contract
from edgeai_common.frames import frame, validate_robot_id
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState, TaskReason
from edgeai_robot_interfaces.srv import Relocalize
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
import yaml

STARTUP = ManageLifecycleNodes.Request.STARTUP


def converged(covariance, max_xy_std, max_yaw_std):
    """Return True when x, y and yaw standard deviations are within the limits."""
    return bool(covariance[0] <= max_xy_std ** 2 and covariance[7] <= max_xy_std ** 2 and
                covariance[35] <= max_yaw_std ** 2)


def pose_in_frame(pose, origin):
    """Express an arena-frame x/y/yaw pose in the frame whose arena pose is origin."""
    if origin is None:
        return dict(pose)
    dx, dy = pose['x'] - origin['x'], pose['y'] - origin['y']
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    return {'x': c * dx + s * dy, 'y': -s * dx + c * dy,
            'yaw': math.atan2(math.sin(pose['yaw'] - origin['yaw']),
                              math.cos(pose['yaw'] - origin['yaw']))}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def scan_match_ratio(ranges, angle_min, angle_increment, range_min, range_max, grid,
                     sensor_pose, radius_cells=2, occupied_threshold=65, max_range=None):
    """
    Fraction of valid scan endpoints that land within radius_cells of an occupied cell.

    grid: dict with width, height, resolution, origin_x, origin_y, origin_yaw and data
    (row-major occupancy values). sensor_pose: (x, y, yaw) of the scan frame in the map.
    Endpoints outside the map count as misses; invalid ranges are skipped.
    Returns (ratio, number of valid endpoints).
    """
    width, height, res = grid['width'], grid['height'], grid['resolution']
    ox, oy, oyaw = grid['origin_x'], grid['origin_y'], grid['origin_yaw']
    data = grid['data']
    cos_o, sin_o = math.cos(-oyaw), math.sin(-oyaw)
    sx, sy, syaw = sensor_pose
    limit = min(range_max, max_range) if max_range else range_max
    hits = valid = 0
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r >= limit:
            continue
        valid += 1
        angle = syaw + angle_min + i * angle_increment
        wx, wy = sx + r * math.cos(angle), sy + r * math.sin(angle)
        gx = cos_o * (wx - ox) - sin_o * (wy - oy)
        gy = sin_o * (wx - ox) + cos_o * (wy - oy)
        cx, cy = int(math.floor(gx / res)), int(math.floor(gy / res))
        found = False
        for dy in range(-radius_cells, radius_cells + 1):
            row = cy + dy
            if row < 0 or row >= height:
                continue
            for dx in range(-radius_cells, radius_cells + 1):
                col = cx + dx
                if 0 <= col < width and data[row * width + col] >= occupied_threshold:
                    found = True
                    break
            if found:
                break
        hits += found
    return (hits / valid if valid else 0.0), valid


def pose_message(x, y, yaw, xy_std, yaw_std, stamp):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    msg.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    msg.pose.covariance[0] = msg.pose.covariance[7] = xy_std ** 2
    msg.pose.covariance[35] = yaw_std ** 2
    return msg


class _Failure(Exception):

    def __init__(self, code, message):
        super().__init__(message)
        self.code = code
        self.message = message


class Localization(Node):
    """Serves relocalize and manages the AMCL and Nav2 lifecycles of one robot."""

    def __init__(self, **kwargs):
        super().__init__('localization', **kwargs)
        p = self.declare_parameter
        self.robot_id = validate_robot_id(p('robot_id', 'robot_1').value)
        run_mode = p('run_mode', 'development').value
        self.require_state = p('require_competition_state', True).value
        config_dir = p('config_dir', '').value or None
        self.start_poses = yaml.safe_load(p('start_poses_yaml', '{}').value) or {}
        parked_id = p('parked_start_pose_id', '').value
        if parked_id and parked_id not in self.start_poses:
            raise ValueError(f'parked_start_pose_id {parked_id!r} is not a start pose')
        origin_id = p('parked_map_origin_pose_id', '').value
        if origin_id and origin_id not in self.start_poses:
            raise ValueError(f'parked_map_origin_pose_id {origin_id!r} is not a start pose')
        self.parked_pose = None
        if parked_id:
            self.parked_pose = pose_in_frame(self.start_poses[parked_id],
                                             self.start_poses.get(origin_id))
        self.max_xy_std = float(p('convergence_max_xy_std_m', 0.10).value)
        self.max_yaw_std = float(p('convergence_max_yaw_std_rad', 0.10).value)
        self.hold_s = float(p('convergence_hold_s', 1.0).value)
        self.hint_xy_std = float(p('default_hint_xy_std_m', 0.10).value)
        self.hint_yaw_std = float(p('default_hint_yaw_std_rad', 0.10).value)
        self.handover_xy_std = float(p('handover_hint_xy_std_m', 0.03).value)
        self.handover_yaw_std = float(p('handover_hint_yaw_std_rad', 0.03).value)
        self.slam_handover_pending = bool(p('slam_handover', False).value)
        self.hint_updates = int(p('hint_nomotion_updates', 10).value)
        self.global_updates = int(p('global_nomotion_updates', 30).value)
        self.update_rate_hz = float(p('nomotion_update_rate_hz', 5.0).value)
        self.activate_navigation = p('activate_navigation', True).value
        self.min_match_ratio = float(p('scan_match_min_ratio', 0.6).value)
        self.match_radius_cells = int(p('scan_match_radius_cells', 2).value)
        self.match_max_range = float(p('scan_match_max_range_m', 6.0).value)
        if not self.require_state and run_mode != 'development':
            raise ValueError('require_competition_state:=false is only allowed in development')

        contract = load_runtime_contract(config_dir)
        self.default_timeout_s = float(contract['timeouts_s']['relocalize_default'])

        self.lock = threading.Lock()
        self.busy = False
        self.competition = None
        self.amcl = None             # (covariance list, monotonic receive time, msg)
        group = ReentrantCallbackGroup()
        self.create_subscription(CompetitionState, '/competition/state', self._on_state,
                                 qos_from_contract(contract, 'control_state'),
                                 callback_group=group)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10,
                                 callback_group=group)
        self.scan = None
        self.grid = None
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data,
                                 callback_group=group)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos,
                                 callback_group=group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        # Last relocalization outcome; robot_agent reports CODE_RELOCALIZATION_FAILED from it.
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.last_outcome = (DiagnosticStatus.OK, 'not relocalized yet')
        self.create_timer(1.0, self._publish_diagnostics, callback_group=group)

        def client(srv_type, name):
            return self.create_client(srv_type, name, callback_group=group)

        self.amcl_manager = client(ManageLifecycleNodes,
                                   'lifecycle_manager_localization/manage_nodes')
        self.amcl_active = client(Trigger, 'lifecycle_manager_localization/is_active')
        self.nav_manager = client(ManageLifecycleNodes,
                                  'lifecycle_manager_navigation/manage_nodes')
        self.nav_active = client(Trigger, 'lifecycle_manager_navigation/is_active')
        self.global_init = client(Empty, 'reinitialize_global_localization')
        self.nomotion = client(Empty, 'request_nomotion_update')
        self.create_service(Relocalize, 'relocalize', self._relocalize, callback_group=group)
        if self.parked_pose is not None:
            self.odom_frame = frame(self.robot_id, 'odom')
            self.base_frame = frame(self.robot_id, 'base_footprint')
            self.tf_broadcaster = TransformBroadcaster(self)
            self.create_timer(0.05, self._publish_parked,
                              callback_group=MutuallyExclusiveCallbackGroup())
            self.get_logger().info(f'{self.robot_id} parked at {parked_id} until relocalized')
        self.get_logger().info(
            f'{self.robot_id} localization ready; start poses: {sorted(self.start_poses)}')

    # ------------------------------------------------------------ helpers
    def _on_state(self, msg):
        self.competition = msg

    def _on_amcl(self, msg):
        self.amcl = ([float(v) for v in msg.pose.covariance], time.monotonic(), msg)

    def _on_scan(self, msg):
        self.scan = msg

    def _on_map(self, msg):
        self.grid = {
            'width': msg.info.width, 'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x, 'origin_y': msg.info.origin.position.y,
            'origin_yaw': yaw_of(msg.info.origin.orientation), 'data': list(msg.data),
        }

    def _scan_match(self, estimate):
        """Project the latest scan with the estimated pose; return the match ratio."""
        scan, grid = self.scan, self.grid
        if scan is None or grid is None:
            raise _Failure(TaskReason.RELOCALIZATION_FAILED, 'no scan or map for the match check')
        try:
            t = self.tf_buffer.lookup_transform(f'{self.robot_id}/base_footprint',
                                                scan.header.frame_id, Time(),
                                                timeout=Duration(seconds=1.0))
        except TransformException as exc:
            raise _Failure(TaskReason.RELOCALIZATION_FAILED,
                           f'no base -> laser transform: {exc}') from exc
        bx = estimate.pose.pose.position.x
        by = estimate.pose.pose.position.y
        byaw = yaw_of(estimate.pose.pose.orientation)
        lx, ly = t.transform.translation.x, t.transform.translation.y
        sensor = (bx + lx * math.cos(byaw) - ly * math.sin(byaw),
                  by + lx * math.sin(byaw) + ly * math.cos(byaw),
                  byaw + yaw_of(t.transform.rotation))
        ratio, valid = scan_match_ratio(
            scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
            grid, sensor, radius_cells=self.match_radius_cells, max_range=self.match_max_range)
        if valid < 20:
            raise _Failure(TaskReason.RELOCALIZATION_FAILED,
                           f'only {valid} valid scan points for the match check')
        return ratio

    def _call(self, client, request, timeout_s):
        if not client.wait_for_service(timeout_sec=min(timeout_s, 5.0)):
            raise _Failure(TaskReason.CONFIGURATION_INVALID,
                           f'service {client.srv_name} unavailable')
        future = client.call_async(request)
        end = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done():
            raise _Failure(TaskReason.RELOCALIZATION_FAILED,
                           f'service {client.srv_name} timed out')
        return future.result()

    def _is_active(self, client):
        if not client.service_is_ready():
            return False
        try:
            return bool(self._call(client, Trigger.Request(), 2.0).success)
        except _Failure:
            return False

    def _startup(self, manager, active_client, name, timeout_s):
        if self._is_active(active_client):
            return
        response = self._call(manager, ManageLifecycleNodes.Request(command=STARTUP), timeout_s)
        if not response.success:
            raise _Failure(TaskReason.RELOCALIZATION_FAILED, f'{name} failed to start')

    def _nomotion_updates(self, count):
        period = 1.0 / self.update_rate_hz
        for _ in range(count):
            self._call(self.nomotion, Empty.Request(), 2.0)
            time.sleep(period)

    def _wait_converged(self, since, deadline):
        """Wait for AMCL poses newer than `since` to stay converged for hold_s."""
        converged_since = None
        while time.monotonic() < deadline:
            sample = self.amcl
            if sample is not None and sample[1] > since:
                if converged(sample[0], self.max_xy_std, self.max_yaw_std):
                    converged_since = converged_since or time.monotonic()
                    if time.monotonic() - converged_since >= self.hold_s:
                        return sample[2]
                else:
                    converged_since = None
            time.sleep(0.05)
        raise _Failure(TaskReason.RELOCALIZATION_FAILED,
                       'AMCL did not converge within the timeout')

    def _check_request(self, request):
        if not self.require_state:
            return
        # Right after start-up the latched CompetitionState may not have arrived yet.
        end = time.monotonic() + 2.0
        while self.competition is None and time.monotonic() < end:
            time.sleep(0.05)
        if self.competition is None:
            raise _Failure(TaskReason.CONFIGURATION_INVALID, 'no CompetitionState received')
        if request.run_id != self.competition.run_id:
            raise _Failure(TaskReason.CONFIGURATION_INVALID, 'run_id mismatch')
        if (request.map_id, request.map_revision) != (self.competition.map_id,
                                                      self.competition.map_revision):
            raise _Failure(TaskReason.CONFIGURATION_INVALID, 'map_id/map_revision mismatch')

    def _publish_diagnostics(self):
        level, message = self.last_outcome
        status = DiagnosticStatus(level=level, name=f'{self.robot_id}/localization',
                                  hardware_id=self.robot_id, message=message)
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(array)

    # ------------------------------------------------------------ service
    def _publish_parked(self):
        pose = self.parked_pose
        if pose is None:
            return
        try:
            odom = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except TransformException:
            return
        # map -> odom = (map -> base at the start pose) * (odom -> base)^-1
        t = odom.transform
        odom_yaw = yaw_of(t.rotation)
        yaw = pose['yaw'] - odom_yaw
        c, s = math.cos(yaw), math.sin(yaw)
        msg = TransformStamped()
        # Valid slightly ahead, like AMCL's transform_tolerance, so lookups at the
        # latest odometry time succeed.
        msg.header.stamp = (Time.from_msg(odom.header.stamp) + Duration(seconds=0.5)).to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = self.odom_frame
        msg.transform.translation.x = pose['x'] - (c * t.translation.x - s * t.translation.y)
        msg.transform.translation.y = pose['y'] - (s * t.translation.x + c * t.translation.y)
        msg.transform.rotation.z = math.sin(yaw / 2.0)
        msg.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(msg)

    def _relocalize(self, request, response):
        with self.lock:
            if self.busy:
                response.success = False
                response.reason = TaskReason(code=TaskReason.CONFIGURATION_INVALID,
                                             detail='a relocalization is already running')
                response.message = response.reason.detail
                return response
            self.busy = True
        started = time.monotonic()
        timeout_s = (request.timeout.sec + request.timeout.nanosec * 1e-9) or \
            self.default_timeout_s
        deadline = started + timeout_s
        try:
            self._check_request(request)
            if self.parked_pose is not None:
                self.parked_pose = None     # AMCL takes over map -> odom from here on
                self.get_logger().info('stopped the parked map -> odom transform')
            self._startup(self.amcl_manager, self.amcl_active, 'AMCL', timeout_s)
            since = time.monotonic()
            stamp = self.get_clock().now().to_msg()
            if request.mode == Relocalize.Request.MODE_KNOWN_START_POSE:
                pose = self.start_poses.get(request.start_pose_id)
                if pose is None:
                    raise _Failure(TaskReason.CONFIGURATION_INVALID,
                                   f'unknown start_pose_id {request.start_pose_id!r}')
                self.initialpose_pub.publish(pose_message(
                    pose['x'], pose['y'], pose['yaw'], self.hint_xy_std, self.hint_yaw_std,
                    stamp))
                self._nomotion_updates(self.hint_updates)
                note = f'known start pose {request.start_pose_id}'
            elif request.mode == Relocalize.Request.MODE_POSE_HINT:
                hint = request.pose_hint
                if hint.header.frame_id != 'map':
                    raise _Failure(TaskReason.CONFIGURATION_INVALID,
                                   'pose_hint must be in the map frame')
                if not any(hint.pose.covariance):
                    q = hint.pose.pose.orientation
                    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    handover = self.slam_handover_pending and (
                        self.competition is None or
                        self.competition.phase == CompetitionState.PHASE_HANDOVER)
                    xy_std, yaw_std = (self.handover_xy_std, self.handover_yaw_std) if \
                        handover else (self.hint_xy_std, self.hint_yaw_std)
                    hint = pose_message(hint.pose.pose.position.x, hint.pose.pose.position.y,
                                        yaw, xy_std, yaw_std, stamp)
                hint.header.stamp = stamp
                self.initialpose_pub.publish(hint)
                self._nomotion_updates(self.hint_updates)
                note = 'pose hint'
            elif request.mode == Relocalize.Request.MODE_GLOBAL:
                self._call(self.global_init, Empty.Request(), 5.0)
                self._nomotion_updates(self.global_updates)
                note = 'global localization (low confidence in a symmetric arena)'
            else:
                raise _Failure(TaskReason.CONFIGURATION_INVALID, f'unknown mode {request.mode}')

            estimate = self._wait_converged(since, deadline)
            ratio = self._scan_match(estimate)
            if ratio < self.min_match_ratio:
                raise _Failure(TaskReason.RELOCALIZATION_FAILED,
                               f'scan does not match the map at the estimate '
                               f'(match {ratio:.2f} < {self.min_match_ratio:.2f})')
            if self.activate_navigation:
                self._startup(self.nav_manager, self.nav_active, 'navigation',
                              max(deadline - time.monotonic(), 5.0))
            response.success = True
            self.slam_handover_pending = False
            response.estimated_pose = estimate
            response.reason = TaskReason(code=TaskReason.NONE, detail='')
            response.message = (f'relocalized by {note} in {time.monotonic() - started:.1f} s, '
                                f'scan match {ratio:.2f}')
        except _Failure as failure:
            response.success = False
            response.reason = TaskReason(code=failure.code, detail=failure.message)
            response.message = failure.message
        finally:
            with self.lock:
                self.busy = False
        if response.success:
            self.last_outcome = (DiagnosticStatus.OK, response.message)
        else:
            self.last_outcome = (DiagnosticStatus.ERROR,
                                 f'relocalize failed: {response.message}')
        self._publish_diagnostics()
        # Separate call sites: rclpy refuses to log from one call site at different
        # severities and raises, which would kill the node on the first failure.
        if response.success:
            self.get_logger().info(f'relocalize mode={request.mode}: {response.message}')
        else:
            self.get_logger().warn(f'relocalize mode={request.mode}: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
