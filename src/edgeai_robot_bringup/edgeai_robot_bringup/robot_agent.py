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
Robot agent: the robot's interface to the edge (P1 plan 2-10 .. 2-14).

Publishes RobotState (10 Hz) and RobotPhaseAck (2 Hz) and serves
execute_navigation_goal and execute_rescue_task (reach and hold, P1 plan 3-04)
on top of Nav2's navigate_to_pose. The robot never times a dwell or decides
that a rescue is complete: the edge rescue_verifier does, and the coordinator
then cancels the action to release the robot. The edge never
calls Nav2 directly. The agent never publishes velocities: it cancels Nav2
and relies on safety_watchdog (ADR-0003) to output zero, then confirms the
stop from odometry before returning a result.
"""

from dataclasses import dataclass
import math
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from edgeai_common.config import load_runtime_contract
from edgeai_common.frames import frame, validate_robot_id
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.action import ExecuteNavigationGoal, ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState,
    EmergencyStop,
    RobotPhaseAck,
    RobotState,
    SystemFault,
    TaskReason,
)
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import (BackUp, ComputePathToPose, DriveOnHeading, FollowPath,
                              NavigateToPose, Spin)
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

MOTION_PHASES = (CompetitionState.PHASE_TASK1, CompetitionState.PHASE_TASK2A,
                 CompetitionState.PHASE_TASK2B)
NavResult = ExecuteNavigationGoal.Result
RescueResult = ExecuteRescueTask.Result
# Terminal states shared by both actions have the same values.
assert (NavResult.TERMINAL_FAILED, NavResult.TERMINAL_CANCELLED, NavResult.TERMINAL_BLOCKED) == \
    (RescueResult.TERMINAL_FAILED, RescueResult.TERMINAL_CANCELLED, RescueResult.TERMINAL_BLOCKED)
# Hysteresis beyond the arrival tolerance before a holding robot re-approaches.
REAPPROACH_MARGIN_M = 0.03
# Largest shift of the aim point from the slot when a straight entry is retried, and the
# shortest entry worth backing out of (a shorter one cannot be re-aimed from roomier space).
AIM_CORRECTION_MAX_M = 0.20
MIN_BACK_OUT_M = 0.05

# Nav2 error codes that mean the robot could not make progress along a valid path.
BLOCKED_ERROR_CODES = (FollowPath.Result.PATIENCE_EXCEEDED,
                       FollowPath.Result.FAILED_TO_MAKE_PROGRESS,
                       FollowPath.Result.NO_VALID_CONTROL)


# --------------------------------------------------------------- pure logic

def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def duration_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def covariance_ok(covariance, max_xy_std, max_yaw_std):
    """Return True when the x, y and yaw standard deviations are within limits."""
    # bool(): ROS covariance arrays are numpy arrays, and a numpy.bool_ assigned to a
    # message bool field aborts the process in the generated C conversion.
    return bool(covariance[0] <= max_xy_std ** 2 and covariance[7] <= max_xy_std ** 2 and
                covariance[35] <= max_yaw_std ** 2)


def map_nav2_failure(status, error_code, error_msg=''):
    """Map a failed navigate_to_pose outcome to (terminal_state, reason_code, detail)."""
    if error_code in BLOCKED_ERROR_CODES:
        return NavResult.TERMINAL_BLOCKED, TaskReason.NAVIGATION_FAILED, \
            f'blocked: Nav2 error {error_code} {error_msg}'.strip()
    if error_code == ComputePathToPose.Result.TIMEOUT:
        return NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_TIMEOUT, \
            f'planner timeout: Nav2 error {error_code}'
    return NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_FAILED, \
        f'Nav2 status {status} error {error_code} {error_msg}'.strip()


def within_tolerance(pose_xy_yaw, goal_xy_yaw, xy_tolerance, yaw_tolerance):
    """Check the final pose; a yaw tolerance of 0 means the heading is ignored."""
    x, y, yaw = pose_xy_yaw
    gx, gy, gyaw = goal_xy_yaw
    if math.hypot(gx - x, gy - y) > xy_tolerance:
        return False
    return yaw_tolerance <= 0.0 or abs(normalize_angle(gyaw - yaw)) <= yaw_tolerance


@dataclass
class ModeInputs:
    estop_active: bool
    motion_allowed: bool
    fatal_fault: bool
    active_kind: str | None   # None, 'navigation' or 'rescue'
    holding: bool
    exploring: bool


def straight_line_move(pose_xy_yaw, target_xy, max_distance, max_lateral):
    """Signed distance along the heading that reaches the target, or None if a turn is needed."""
    dx, dy = target_xy[0] - pose_xy_yaw[0], target_xy[1] - pose_xy_yaw[1]
    yaw = pose_xy_yaw[2]
    along = dx * math.cos(yaw) + dy * math.sin(yaw)
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    if abs(along) > max_distance or abs(lateral) > max_lateral:
        return None
    return along


def alignment_turn(pose_xy_yaw, target_xy, max_distance, max_turn):
    """
    Small in-place turn that points the front (target ahead) or rear (behind) at it.

    The approach heading is only within Nav2's yaw tolerance, so a slot entered
    straight from its approach (forward or in reverse) can lie too far sideways for
    the straight move; this turn happens at the roomy approach, not next to the slot.
    Returns None when the target is too far or the turn larger than max_turn.
    """
    dx, dy = target_xy[0] - pose_xy_yaw[0], target_xy[1] - pose_xy_yaw[1]
    yaw = pose_xy_yaw[2]
    if math.hypot(dx, dy) > max_distance:
        return None
    ahead = dx * math.cos(yaw) + dy * math.sin(yaw) >= 0.0
    turn = (math.atan2(dy, dx) if ahead else math.atan2(-dy, -dx)) - yaw
    turn = math.atan2(math.sin(turn), math.cos(turn))
    return turn if abs(turn) <= max_turn else None


def travelled_along(start_xy, pose_xy_yaw):
    """Signed distance driven along the current heading since start_xy (negative: reversed)."""
    dx, dy = pose_xy_yaw[0] - start_xy[0], pose_xy_yaw[1] - start_xy[1]
    return dx * math.cos(pose_xy_yaw[2]) + dy * math.sin(pose_xy_yaw[2])


def compensated_aim(target_xy, aim_xy, end_xy, max_shift):
    """
    Aim point for the next straight entry: the last aim moved against its miss.

    A straight move holds the odometry heading, so it drifts the same way every time
    (about 4 degrees in the midterm C1 corridor); aiming beside the slot by the last
    miss cancels it. The total shift from the slot is capped at max_shift.
    """
    x = aim_xy[0] + target_xy[0] - end_xy[0]
    y = aim_xy[1] + target_xy[1] - end_xy[1]
    shift = math.hypot(x - target_xy[0], y - target_xy[1])
    if shift > max_shift:
        scale = max_shift / shift
        x = target_xy[0] + (x - target_xy[0]) * scale
        y = target_xy[1] + (y - target_xy[1]) * scale
    return (x, y)


def select_mode(inputs: ModeInputs):
    """RobotState.mode by the priority table of the P1 plan (2-10)."""
    if inputs.estop_active:
        return RobotState.MODE_ESTOP
    if not inputs.motion_allowed:
        return RobotState.MODE_SAFE
    if inputs.fatal_fault:
        return RobotState.MODE_ERROR
    if inputs.active_kind == 'rescue' and inputs.holding:
        return RobotState.MODE_HOLDING
    if inputs.active_kind == 'navigation' and inputs.exploring:
        return RobotState.MODE_EXPLORING
    if inputs.active_kind is not None:
        return RobotState.MODE_NAVIGATING
    return RobotState.MODE_STANDBY


class _GoalEnd(Exception):
    """Ends an execute loop with a terminal state and reason."""

    def __init__(self, terminal_state, code, detail):
        super().__init__(detail)
        self.terminal_state = terminal_state
        self.code = code
        self.detail = detail


# --------------------------------------------------------------------- node

class RobotAgent(Node):
    """Robot-side agent serving the shared P1 interfaces."""

    def __init__(self, **kwargs):
        super().__init__('robot_agent', **kwargs)
        p = self.declare_parameter
        self.robot_id = validate_robot_id(p('robot_id', 'robot_1').value)
        self.run_mode = p('run_mode', 'development').value
        self.require_state = p('require_competition_state', True).value
        self.localization_mode = p('localization_mode', 'amcl').value
        config_dir = p('config_dir', '').value or None
        # Limits for "still localized", not for relocalization convergence (plan 3-02
        # uses 0.10 m). AMCL reports its particle spread: in simulation it grew to about
        # 0.31 m / 0.33 rad during a 4.7 m drive while the true error stayed near 0.1 m.
        self.max_xy_std = float(p('localized_max_xy_std_m', 0.5).value)
        self.max_yaw_std = float(p('localized_max_yaw_std_rad', 0.6).value)
        self.slam_variance = float(p('slam_pose_variance', 0.0025).value)
        self.tf_stale_s = float(p('tf_stale_s', 0.5).value)
        self.sensor_timeout_s = float(p('sensor_timeout_s', 1.0).value)
        self.default_xy_tolerance = float(p('default_xy_tolerance_m', 0.10).value)
        self.blocked_after_s = float(p('blocked_after_s', 5.0).value)
        self.stop_linear_eps = float(p('stop_linear_eps_mps', 0.01).value)
        self.stop_angular_eps = float(p('stop_angular_eps_radps', 0.02).value)
        # Rescue completion does not depend on orientation (rule 3.3): approach with the
        # heading of travel so Nav2 does not spend time turning to the zone's yaw.
        self.rescue_ignore_orientation = p('rescue_ignore_orientation', True).value
        # A robot pushed along its heading drives straight back with the Nav2 BackUp or
        # DriveOnHeading behavior: turning on the spot to face the target can swing the
        # long rear overhang into a nearby wall and trip the collision monitor.
        # The midterm launch passes 1.0 m (robot_stack rescue_straight_max_m): it covers
        # every approach -> slot leg (0.45-0.80 m) plus the 0.10 m approach tolerance,
        # so robots enter slots straight instead of being turned next to walls/stands.
        self.straight_max_m = float(p('rescue_straight_max_m', 0.6).value)
        # Largest in-place turn that lines the robot up with the slot before that move.
        self.align_max_turn = float(p('rescue_align_max_turn_rad', 0.6).value)
        self.straight_speed = float(p('rescue_straight_speed_mps', 0.08).value)
        # A straight entry that ends outside the tolerance backs out along its own line
        # and is retried with the miss compensated, instead of letting Nav2 turn the
        # robot next to the slot (C1's second slot has 0.32-0.43 m to the wall and the
        # parked peer; turning needs 0.43 m).
        self.straight_retries = int(p('rescue_straight_retries', 2).value)
        # Arrival tolerance for cooperative rescues when larger than the coordinator's
        # (0 = off). At C1's slots near the east wall AMCL jumps by several cm, and three
        # straight entries ending 0.08-0.13 m off left Nav2 to turn the robot by the wall,
        # which it could not do. The midterm C1 slots lie 0.13-0.16 m inside the 0.7 m
        # radius, so the midterm launch accepts 0.10 m there; S1's slot is only 0.09 m
        # inside its radius and keeps the coordinator's tolerance.
        self.cooperative_accept_m = float(p('rescue_cooperative_accept_m', 0.0).value)

        if not self.require_state and self.run_mode != 'development':
            raise ValueError('require_competition_state:=false is only allowed in development')
        if self.localization_mode not in ('amcl', 'slam'):
            raise ValueError(f'localization_mode must be amcl or slam, not '
                             f'{self.localization_mode!r}')

        contract = load_runtime_contract(config_dir)
        timeouts = contract['timeouts_s']
        rates = contract['rates_hz']
        self.edge_timeout_s = float(timeouts['edge_connection'])
        self.navigation_default_s = float(timeouts['navigation_default'])
        self.safe_stop_ack_s = float(timeouts['safe_stop_ack'])
        self.tf_lookup_s = float(timeouts['tf_lookup'])

        self.lock = threading.RLock()
        self.competition = None
        self.last_state_time = None
        self.estop_generation = None
        self.estop_active = False
        self.active_goal = None          # (kind, id, purpose)
        self.abort_request = None        # (reason code, detail)
        self.amcl_covariance = None
        self.odom_twist = (0.0, 0.0)
        self.last_odom_time = None
        self.last_scan_time = None
        self.navigation_available = False
        self.collision_stop_since = None
        self.command_timeout_fault = False
        self.relocalize_failed = False
        self.holding = False
        self.started = time.monotonic()
        self.pose = None                 # (x, y, yaw, stamp_seconds)
        self.nav_distance_remaining = 0.0

        group = ReentrantCallbackGroup()
        control_qos = qos_from_contract(contract, 'control_state')
        self.create_subscription(CompetitionState, '/competition/state', self._on_state,
                                 control_qos, callback_group=group)
        self.create_subscription(EmergencyStop, '/competition/estop', self._on_estop,
                                 control_qos, callback_group=group)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10, callback_group=group)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data,
                                 callback_group=group)
        # Transient local like AMCL's publisher: AMCL publishes only while converging (a
        # parked robot then stands still), and a late DDS match would otherwise miss the
        # only poses and leave the robot "not localized" (midterm TASK2A, robot_2).
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL),
                                 callback_group=group)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self._on_collision_state, 10, callback_group=group)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diagnostics, 10,
                                 callback_group=group)
        self.rescue_server = ActionServer(
            self, ExecuteRescueTask, 'execute_rescue_task',
            execute_callback=self._execute_rescue,
            goal_callback=self._accept_rescue,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        robot_qos = qos_from_contract(contract, 'robot_state')
        self.state_pub = self.create_publisher(RobotState, 'state', robot_qos)
        self.ack_pub = self.create_publisher(RobotPhaseAck, 'phase_ack', robot_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                       callback_group=group)
        self.backup_client = ActionClient(self, BackUp, 'backup', callback_group=group)
        self.spin_client = ActionClient(self, Spin, 'spin', callback_group=group)
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading',
                                         callback_group=group)
        self.nav_active_client = self.create_client(
            Trigger, 'lifecycle_manager_navigation/is_active', callback_group=group)
        self.nav_server = ActionServer(
            self, ExecuteNavigationGoal, 'execute_navigation_goal',
            execute_callback=self._execute_navigation,
            goal_callback=self._accept_navigation,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)

        # Output timers run one at a time: overlapping callbacks would publish on the
        # same publisher from several threads at once, which aborts the process.
        timers = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / rates['robot_state'], self._publish_state, callback_group=timers)
        self.create_timer(1.0 / rates['robot_phase_ack_until_accepted'], self._publish_ack,
                          callback_group=timers)
        self.create_timer(1.0, self._poll_navigation, callback_group=timers)
        self.get_logger().info(f'{self.robot_id} robot_agent ready ({self.localization_mode})')

    # ------------------------------------------------------------- inputs
    def _on_state(self, msg):
        with self.lock:
            previous = self.competition
            self.competition = msg
            self.last_state_time = time.monotonic()
            changed = previous is None or previous.phase_sequence != msg.phase_sequence
            if changed and self.active_goal and msg.phase not in MOTION_PHASES:
                self.abort_request = (TaskReason.PHASE_CHANGED, f'phase changed to {msg.phase}')

    def _on_estop(self, msg):
        with self.lock:
            if self.estop_generation is not None and msg.generation <= self.estop_generation:
                return
            self.estop_generation = msg.generation
            self.estop_active = msg.active
            if msg.active and self.active_goal:
                self.abort_request = (TaskReason.SAFETY_STOP, f'emergency stop: {msg.reason}')

    def _on_odom(self, msg):
        self.odom_twist = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.last_odom_time = time.monotonic()

    def _on_scan(self, _msg):
        self.last_scan_time = time.monotonic()

    def _on_amcl(self, msg):
        self.amcl_covariance = [float(v) for v in msg.pose.covariance]

    def _on_collision_state(self, msg):
        if msg.action_type == CollisionMonitorState.STOP:
            if self.collision_stop_since is None:
                self.collision_stop_since = time.monotonic()
        else:
            self.collision_stop_since = None

    def _on_diagnostics(self, msg):
        for status in msg.status:
            if status.name == f'{self.robot_id}/localization':
                self.relocalize_failed = status.level == DiagnosticStatus.ERROR

    def _poll_navigation(self):
        if not self.nav_active_client.service_is_ready():
            self.navigation_available = False
            return
        future = self.nav_active_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_navigation_active)

    def _on_navigation_active(self, future):
        try:
            self.navigation_available = bool(future.result().success)
        except Exception:  # noqa: BLE001 - a failed call simply means unavailable
            self.navigation_available = False

    # ------------------------------------------------------------ derived
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _update_pose(self, wait=True):
        """Refresh the map pose from TF; timers pass wait=False and never block."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', frame(self.robot_id, 'base_footprint'), Time(),
                timeout=Duration(seconds=self.tf_lookup_s if wait else 0.0))
        except TransformException as error:
            if self.pose is not None:
                self.get_logger().warning(f'map -> base_footprint lookup failed: {error}',
                                          throttle_duration_sec=5.0)
            return
        tr = t.transform
        stamp = t.header.stamp.sec + t.header.stamp.nanosec * 1e-9
        self.pose = (tr.translation.x, tr.translation.y, yaw_from_quaternion(tr.rotation), stamp)

    def _tf_fresh(self):
        return self.pose is not None and self._now_s() - self.pose[3] <= self.tf_stale_s

    def _localization_ok(self):
        # AMCL publishes amcl_pose only while active. Once it does (always in amcl mode,
        # after the SLAM -> AMCL handover in slam mode) its covariance decides.
        if self.amcl_covariance is not None:
            return covariance_ok(self.amcl_covariance, self.max_xy_std, self.max_yaw_std)
        return self.localization_mode == 'slam'

    def _localized(self):
        return bool(self._tf_fresh() and self._localization_ok())

    def _edge_connected(self):
        return (self.last_state_time is not None and
                time.monotonic() - self.last_state_time <= self.edge_timeout_s)

    def _motion_allowed(self):
        if self.estop_active:
            return False
        if not self.require_state:
            return True
        return (self.competition is not None and self.competition.phase in MOTION_PHASES and
                self._edge_connected())

    def _sensor_ok(self):
        if self.last_scan_time is None:
            return time.monotonic() - self.started < 5.0
        return time.monotonic() - self.last_scan_time <= self.sensor_timeout_s

    def _stopped(self):
        linear, angular = self.odom_twist
        return bool(abs(linear) <= self.stop_linear_eps and
                    abs(angular) <= self.stop_angular_eps)

    def _fault(self, code, severity, detail):
        return SystemFault(source=f'{self.robot_id}/robot_agent', code=code,
                           severity=severity, detail=detail)

    def _faults(self):
        faults = []
        if self.pose is not None and not self._tf_fresh():
            faults.append(self._fault(SystemFault.CODE_TF_STALE, SystemFault.SEVERITY_ERROR,
                                      'map -> base_footprint is stale'))
        if self.amcl_covariance is not None and \
                not self._localization_ok():
            faults.append(self._fault(SystemFault.CODE_LOCALIZATION_LOST,
                                      SystemFault.SEVERITY_ERROR,
                                      'AMCL covariance above threshold'))
        if not self.navigation_available:
            faults.append(self._fault(SystemFault.CODE_NAVIGATION_ABORTED,
                                      SystemFault.SEVERITY_WARN, 'Nav2 is not active'))
        if self.command_timeout_fault:
            faults.append(self._fault(SystemFault.CODE_COMMAND_TIMEOUT,
                                      SystemFault.SEVERITY_ERROR,
                                      'robot did not stop within safe_stop_ack'))
        if self.relocalize_failed:
            faults.append(self._fault(SystemFault.CODE_RELOCALIZATION_FAILED,
                                      SystemFault.SEVERITY_ERROR, 'last relocalize failed'))
        if self.estop_active:
            faults.append(self._fault(SystemFault.CODE_ESTOP_ACTIVE, SystemFault.SEVERITY_WARN,
                                      'emergency stop active'))
        if self.require_state and self.last_state_time is not None and \
                not self._edge_connected():
            faults.append(self._fault(SystemFault.CODE_EDGE_CONNECTION_LOST,
                                      SystemFault.SEVERITY_ERROR,
                                      'no CompetitionState heartbeat'))
        if not self._sensor_ok():
            faults.append(self._fault(SystemFault.CODE_SENSOR_UNAVAILABLE,
                                      SystemFault.SEVERITY_ERROR, 'scan: no data'))
        return faults

    # ---------------------------------------------------------- goal checks
    def _rejection(self, run_id, map_id, map_revision, pose_frame):
        if self.active_goal is not None:
            return 'another goal is active; cancel it first'
        if self.require_state:
            if self.competition is None:
                return 'no CompetitionState received'
            if run_id != self.competition.run_id:
                return 'run_id mismatch'
            if (map_id, map_revision) != (self.competition.map_id,
                                          self.competition.map_revision):
                return 'map_id/map_revision mismatch'
        if not self._motion_allowed():
            return 'motion not allowed in the current phase or safety state'
        self._update_pose()
        if not self._localized():
            return 'robot is not localized'
        if not self.navigation_available:
            return 'Nav2 is not active'
        if pose_frame != 'map':
            return 'goal pose must be in the map frame'
        return None

    def _accept_navigation(self, goal):
        with self.lock:
            reason = self._rejection(goal.run_id, goal.map_id, goal.map_revision,
                                     goal.goal_pose.header.frame_id)
            if reason is None:
                self.active_goal = ('navigation', goal.request_id, goal.purpose)
                self.abort_request = None
                return GoalResponse.ACCEPT
        self.get_logger().warn(f'rejected navigation goal {goal.request_id}: {reason}')
        return GoalResponse.REJECT

    # ----------------------------------------------------------- navigation
    def _send_to_nav2(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_FAILED,
                           'navigate_to_pose server unavailable')
        goal = NavigateToPose.Goal()
        goal.pose = pose
        handle = self._send_goal(self.nav_client, goal, self._on_nav_feedback)
        if handle is None:
            raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_FAILED,
                           'Nav2 rejected the goal')
        return handle

    @staticmethod
    def _send_goal(client, goal, feedback_callback=None):
        """Send a goal and return its handle, or None when it is not accepted in time."""
        future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        return handle if handle is not None and handle.accepted else None

    def _on_nav_feedback(self, feedback_msg):
        self.nav_distance_remaining = feedback_msg.feedback.distance_remaining

    def _follow(self, goal_handle, nav_handle, start, timeout, cancel_reason, feedback):
        """Wait for Nav2 while handling cancel, interrupts, timeout and blocking."""
        result_future = nav_handle.get_result_async()
        last_feedback = 0.0
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                raise _GoalEnd(NavResult.TERMINAL_CANCELLED, cancel_reason(),
                               'cancel requested')
            with self.lock:
                request = self.abort_request
            if request is not None:
                raise _GoalEnd(NavResult.TERMINAL_CANCELLED, request[0], request[1])
            self._update_pose()
            if not self._localized():
                raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.LOCALIZATION_LOST,
                               'localization lost during navigation')
            elapsed = time.monotonic() - start
            if elapsed > timeout:
                raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_TIMEOUT,
                               f'navigation timeout after {timeout:.0f} s')
            # The collision monitor reports state changes only, so a stop that began
            # before this goal is timed from the goal's start.
            stop_since = self.collision_stop_since
            if stop_since is not None and \
                    time.monotonic() - max(stop_since, start) > self.blocked_after_s:
                raise _GoalEnd(NavResult.TERMINAL_BLOCKED, TaskReason.NAVIGATION_FAILED,
                               'blocked: collision monitor stopped the robot')
            if elapsed - last_feedback >= 0.5:
                last_feedback = elapsed
                feedback(elapsed)
            time.sleep(0.05)
        return result_future.result()

    def _cancel_nav2(self, nav_handle):
        if nav_handle is None:
            return
        future = nav_handle.cancel_goal_async()
        end = time.monotonic() + self.safe_stop_ack_s
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)

    def _wait_stopped(self):
        """Wait until odometry reports standstill; return False after safe_stop_ack."""
        end = time.monotonic() + self.safe_stop_ack_s
        while time.monotonic() < end:
            if self._stopped():
                return True
            time.sleep(0.05)
        return self._stopped()

    def _execute_navigation(self, goal_handle):
        goal = goal_handle.request
        xy_tolerance = goal.xy_tolerance_m if goal.xy_tolerance_m > 0.0 else \
            self.default_xy_tolerance
        timeout = duration_seconds(goal.timeout) or self.navigation_default_s
        target = (goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y,
                  yaw_from_quaternion(goal.goal_pose.pose.orientation))
        start = time.monotonic()
        self.nav_distance_remaining = 0.0
        nav_handle = None
        try:
            for attempt in (1, 2):
                nav_handle = self._send_to_nav2(goal.goal_pose)
                outcome = self._follow(goal_handle, nav_handle, start, timeout,
                                       lambda: TaskReason.USER_REQUEST,
                                       lambda elapsed: self._navigation_feedback(goal_handle,
                                                                                 elapsed))
                nav_handle = None
                if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                    raise _GoalEnd(*map_nav2_failure(outcome.status, outcome.result.error_code,
                                                     outcome.result.error_msg))
                self._update_pose()
                if self.pose is not None and within_tolerance(
                        self.pose[:3], target, xy_tolerance, goal.yaw_tolerance_rad):
                    raise _GoalEnd(NavResult.TERMINAL_SUCCEEDED, TaskReason.NONE,
                                   f'goal reached (attempt {attempt})')
            raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_FAILED,
                           'outside the requested tolerance after a precise approach')
        except _GoalEnd as end:
            result = NavResult()
            result.success = end.terminal_state == NavResult.TERMINAL_SUCCEEDED
            self._finish(goal_handle, nav_handle, end, result)
            result.completed_at = self.get_clock().now().to_msg()
            return self._complete(goal_handle, result, result.success, end)

    def _navigation_feedback(self, goal_handle, elapsed):
        feedback = ExecuteNavigationGoal.Feedback()
        feedback.execution_state = ExecuteNavigationGoal.Feedback.STATE_NAVIGATING
        feedback.distance_remaining_m = float(self.nav_distance_remaining)
        feedback.elapsed = DurationMsg(sec=int(elapsed), nanosec=int((elapsed % 1.0) * 1e9))
        goal_handle.publish_feedback(feedback)

    # --------------------------------------------------------------- rescue
    def _accept_rescue(self, goal):
        with self.lock:
            reason = self._rejection(goal.run_id, goal.map_id, goal.map_revision,
                                     goal.rescue_pose.header.frame_id)
            if reason is None and goal.arrival_xy_tolerance_m <= 0.0:
                reason = 'arrival_xy_tolerance_m must be positive'
            if reason is None:
                self.active_goal = ('rescue', goal.task_id, None)
                self.abort_request = None
                self.holding = False
                return GoalResponse.ACCEPT
        self.get_logger().warn(f'rejected rescue task {goal.task_id}: {reason}')
        return GoalResponse.REJECT

    def _release_reason(self):
        """Reason for a cancel of a rescue action (ROS 2 cancels carry no reason)."""
        if self.estop_active:
            return TaskReason.SAFETY_STOP
        if self._motion_allowed():
            return TaskReason.RELEASED_BY_COORDINATOR
        return TaskReason.PHASE_CHANGED

    def _distance_to(self, target):
        if self.pose is None:
            return float('inf')
        return math.hypot(target[0] - self.pose[0], target[1] - self.pose[1])

    def _rescue_feedback(self, goal_handle, state, target):
        feedback = ExecuteRescueTask.Feedback()
        feedback.execution_state = state
        feedback.distance_to_rescue_pose_m = float(self._distance_to(target))
        feedback.message = 'holding' if state == ExecuteRescueTask.Feedback.STATE_HOLDING \
            else 'navigating'
        goal_handle.publish_feedback(feedback)

    def _approach_pose(self, pose, target):
        """Rescue goal for Nav2: optionally replace the yaw by the heading of travel."""
        if not self.rescue_ignore_orientation:
            return pose
        self._update_pose()
        approach = PoseStamped()
        approach.header = pose.header
        approach.pose.position = pose.pose.position
        if self.pose is not None and self._distance_to(target) > 0.05:
            yaw = math.atan2(target[1] - self.pose[1], target[0] - self.pose[0])
        else:
            yaw = self.pose[2] if self.pose is not None else 0.0
        approach.pose.orientation.z = math.sin(yaw / 2.0)
        approach.pose.orientation.w = math.cos(yaw / 2.0)
        return approach

    def _drive_straight(self, goal_handle, distance, target, start):
        """Drive `distance` metres along the heading (negative: back up); True on success."""
        client, action = (self.drive_client, DriveOnHeading) if distance > 0.0 else \
            (self.backup_client, BackUp)
        if not client.wait_for_server(timeout_sec=0.5):
            return False
        goal = action.Goal()
        goal.target.x = abs(distance)       # BackUp reverses on its own
        goal.speed = self.straight_speed
        goal.time_allowance.sec = int(abs(distance) / self.straight_speed * 2.0) + 5
        handle = self._send_goal(client, goal)
        if handle is None:
            return False
        try:
            outcome = self._follow(
                goal_handle, handle, start, self.navigation_default_s, self._release_reason,
                lambda _elapsed: self._rescue_feedback(
                    goal_handle, ExecuteRescueTask.Feedback.STATE_NAVIGATING, target))
        except _GoalEnd:
            self._cancel_nav2(handle)
            raise
        return outcome.status == GoalStatus.STATUS_SUCCEEDED

    def _spin(self, goal_handle, turn, target, start):
        """Turn in place by `turn` rad (Nav2 spin behaviour); True on success."""
        if not self.spin_client.wait_for_server(timeout_sec=0.5):
            return False
        goal = Spin.Goal()
        goal.target_yaw = float(turn)
        goal.time_allowance.sec = 10
        handle = self._send_goal(self.spin_client, goal)
        if handle is None:
            return False
        try:
            outcome = self._follow(
                goal_handle, handle, start, self.navigation_default_s, self._release_reason,
                lambda _elapsed: self._rescue_feedback(
                    goal_handle, ExecuteRescueTask.Feedback.STATE_NAVIGATING, target))
        except _GoalEnd:
            self._cancel_nav2(handle)
            raise
        return outcome.status == GoalStatus.STATUS_SUCCEEDED

    def _settle(self, timeout_s=2.0):
        """
        Wait for standstill, then for a map pose stamped after it.

        A fixed short sleep is not enough when the simulation runs slowly (load of 30 with
        Gazebo and RViz open): the pose read right after a turn or move was still stale.
        """
        deadline = time.monotonic() + timeout_s
        time.sleep(0.2)
        while not self._stopped() and time.monotonic() < deadline:
            time.sleep(0.05)
        stopped_at = self._now_s()
        while time.monotonic() < deadline:
            self._update_pose()
            if self.pose is not None and self.pose[3] >= stopped_at:
                return
            time.sleep(0.05)

    def _enter_straight(self, goal_handle, aim, target, tolerance, start):
        """Small turn if needed, then one straight move at `aim`; False if not possible."""
        distance = straight_line_move(self.pose[:3], aim, self.straight_max_m,
                                      0.75 * tolerance)
        turn = None if distance is not None else alignment_turn(
            self.pose[:3], aim, self.straight_max_m, self.align_max_turn)
        if turn is not None and self._spin(goal_handle, turn, target, start):
            self._settle()
            distance = straight_line_move(self.pose[:3], aim, self.straight_max_m,
                                          0.75 * tolerance)
        if distance is None or not self._drive_straight(goal_handle, distance, target, start):
            return False
        self._settle()
        return True

    def _approach(self, goal_handle, pose, target, tolerance, accept=None):
        """
        Navigate until within `accept` (default the tolerance); position only (rule 3.3).

        Straight entries still aim with `tolerance`; `accept` only decides arrival.
        """
        accept = max(tolerance, accept or 0.0)
        start = time.monotonic()
        self._update_pose()
        if self.pose is not None and self._distance_to(target) > accept:
            aim = target
            for retry in range(self.straight_retries + 1):
                # Each entry gets the whole time budget: one back-out and re-entry takes
                # about 30 s of wall time when the simulation runs at half speed.
                start = time.monotonic()
                origin = self.pose[:2]
                if not self._enter_straight(goal_handle, aim, target, tolerance, start):
                    break
                if self._distance_to(target) <= accept:
                    return 0
                back = travelled_along(origin, self.pose[:3])
                if retry == self.straight_retries or abs(back) < MIN_BACK_OUT_M:
                    break
                # Missed: back out along the entry line (no turn next to the slot), then
                # aim again from the roomy approach with the miss compensated.
                self.get_logger().info(
                    f'straight entry ended {self._distance_to(target):.3f} m from the slot; '
                    f'backing out {abs(back):.2f} m to retry')
                aim = compensated_aim(target, aim, self.pose[:2], AIM_CORRECTION_MAX_M)
                if not self._drive_straight(goal_handle, -back, target, start):
                    break
                self._settle()
            start = time.monotonic()
        for attempt in (1, 2):
            nav_handle = self._send_to_nav2(self._approach_pose(pose, target))
            try:
                outcome = self._follow(
                    goal_handle, nav_handle, start, self.navigation_default_s,
                    self._release_reason,
                    lambda _elapsed: self._rescue_feedback(
                        goal_handle, ExecuteRescueTask.Feedback.STATE_NAVIGATING, target))
            except _GoalEnd:
                self._cancel_nav2(nav_handle)
                raise
            if outcome.status != GoalStatus.STATUS_SUCCEEDED:
                raise _GoalEnd(*map_nav2_failure(outcome.status, outcome.result.error_code,
                                                 outcome.result.error_msg))
            self._update_pose()
            if self._distance_to(target) <= accept:
                return attempt
        raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.NAVIGATION_FAILED,
                       'outside the arrival tolerance after a precise approach')

    def _execute_rescue(self, goal_handle):
        goal = goal_handle.request
        tolerance = goal.arrival_xy_tolerance_m
        accept = (max(tolerance, self.cooperative_accept_m)
                  if goal.target_type == ExecuteRescueTask.Goal.TYPE_COOPERATIVE else tolerance)
        hold_timeout = duration_seconds(goal.hold_timeout)
        target = (goal.rescue_pose.pose.position.x, goal.rescue_pose.pose.position.y)
        self.nav_distance_remaining = 0.0
        first_arrival = None
        try:
            while True:
                self._approach(goal_handle, goal.rescue_pose, target, tolerance, accept)
                first_arrival = first_arrival or time.monotonic()
                self.holding = True
                last_feedback = 0.0
                while True:
                    if goal_handle.is_cancel_requested:
                        raise _GoalEnd(NavResult.TERMINAL_CANCELLED, self._release_reason(),
                                       'released by the coordinator')
                    with self.lock:
                        request = self.abort_request
                    if request is not None:
                        raise _GoalEnd(NavResult.TERMINAL_CANCELLED, request[0], request[1])
                    self._update_pose()
                    if not self._localized():
                        raise _GoalEnd(NavResult.TERMINAL_FAILED, TaskReason.LOCALIZATION_LOST,
                                       'localization lost while holding')
                    if hold_timeout > 0.0 and time.monotonic() - first_arrival >= hold_timeout:
                        raise _GoalEnd(RescueResult.TERMINAL_HOLD_TIMEOUT,
                                       TaskReason.HOLD_TIMEOUT, 'hold timeout')
                    # Re-approach once the push is over, so the move starts from rest.
                    if self._distance_to(target) > accept + REAPPROACH_MARGIN_M and \
                            self._motion_allowed() and self._stopped():
                        self.holding = False
                        self.get_logger().info('pushed out of the rescue pose; re-approaching')
                        break
                    now = time.monotonic()
                    if now - last_feedback >= 0.5:
                        last_feedback = now
                        self._rescue_feedback(goal_handle,
                                              ExecuteRescueTask.Feedback.STATE_HOLDING, target)
                    time.sleep(0.1)
        except _GoalEnd as end:
            result = RescueResult()
            self._finish(goal_handle, None, end, result)
            result.ended_at = self.get_clock().now().to_msg()
            return self._complete(goal_handle, result, False, end)

    # --------------------------------------------------------------- common
    def _finish(self, goal_handle, nav_handle, end, result):
        """Stop Nav2, confirm standstill and fill the fields both results share."""
        self._cancel_nav2(nav_handle)
        stopped = self._wait_stopped()
        self.command_timeout_fault = not stopped
        if not stopped:
            self.get_logger().error('robot did not stop within safe_stop_ack after the goal')
        result.terminal_state = end.terminal_state
        result.reason = TaskReason(code=end.code, detail=end.detail)
        self._update_pose()
        result.final_pose.header.frame_id = 'map'
        result.final_pose.header.stamp = self.get_clock().now().to_msg()
        if self.pose is not None:
            x, y, yaw, _ = self.pose
            result.final_pose.pose.position.x, result.final_pose.pose.position.y = x, y
            result.final_pose.pose.orientation.z = math.sin(yaw / 2.0)
            result.final_pose.pose.orientation.w = math.cos(yaw / 2.0)

    def _complete(self, goal_handle, result, success, end):
        """Release the goal slot and set the terminal state (rclpy sends `result` now)."""
        with self.lock:
            self.active_goal = None
            self.abort_request = None
            self.holding = False
        if goal_handle.is_cancel_requested:
            goal_handle.canceled(result)
        elif success:
            goal_handle.succeed(result)
        else:
            goal_handle.abort(result)
        self.get_logger().info(f'goal ended: {end.detail}')
        return result

    # ------------------------------------------------------------- output
    def _publish_state(self):
        self._update_pose(wait=False)
        with self.lock:
            msg = RobotState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.run_id = self.competition.run_id if self.competition else ''
            msg.robot_id = self.robot_id
            faults = self._faults()
            active = self.active_goal
            msg.mode = select_mode(ModeInputs(
                estop_active=self.estop_active,
                motion_allowed=self._motion_allowed(),
                fatal_fault=any(f.severity == SystemFault.SEVERITY_FATAL for f in faults),
                active_kind=active[0] if active else None,
                holding=self.holding,
                exploring=bool(active) and
                active[2] == ExecuteNavigationGoal.Goal.PURPOSE_EXPLORATION))
            if self.pose is not None:
                x, y, yaw, _ = self.pose
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            if self.amcl_covariance is not None:
                msg.pose.covariance = self.amcl_covariance
            else:
                msg.pose.covariance[0] = msg.pose.covariance[7] = self.slam_variance
                msg.pose.covariance[35] = self.slam_variance
            msg.velocity.twist.linear.x, msg.velocity.twist.angular.z = self.odom_twist
            msg.localized = self._localized()
            msg.navigation_available = self.navigation_available
            msg.battery_percentage = float('nan')
            msg.active_task_id = active[1] if active else ''
            msg.faults = faults
        self.state_pub.publish(msg)

    def _publish_ack(self):
        with self.lock:
            if self.competition is None:
                return
            motion_phase = self.competition.phase in MOTION_PHASES
            msg = RobotPhaseAck()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.run_id = self.competition.run_id
            msg.robot_id = self.robot_id
            msg.phase_sequence = self.competition.phase_sequence
            msg.phase = self.competition.phase
            msg.phase_received = True
            msg.actions_cancelled = motion_phase or self.active_goal is None
            # Outside motion phases safety_watchdog outputs zero regardless of inputs.
            msg.zero_velocity_commanded = (not motion_phase) or self.active_goal is None
            msg.safe = self._stopped()
            msg.localized = self._localized()
            msg.ready_for_phase = msg.localized and (
                self.navigation_available if motion_phase else msg.safe)
            msg.faults = self._faults()
        self.ack_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotAgent()
    executor = MultiThreadedExecutor(num_threads=6)
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
