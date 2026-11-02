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
Kinematic stand-in for robot_agent (see the package README).

It serves the same interfaces with the same semantics as the real agent so P2
and P4 can develop against it: RobotState at 10 Hz, RobotPhaseAck, the
execute_navigation_goal and execute_rescue_task actions ("reach and hold"; the
robot never times a dwell or declares a rescue complete) and relocalize. The
pose is integrated in the map frame (turn towards the target, then drive).
Failures are injected through parameters.
"""

import math
import threading
import time

from builtin_interfaces.msg import Duration as DurationMsg
from edgeai_common.config import load_runtime_contract, resolve_parameters
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
from edgeai_robot_interfaces.srv import Relocalize
from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

MOTION_PHASES = (CompetitionState.PHASE_TASK1, CompetitionState.PHASE_TASK2A,
                 CompetitionState.PHASE_TASK2B)
LOOP_HZ = 20.0


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def duration_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def point_in_polygon(x, y, polygon):
    inside = False
    for i in range(len(polygon)):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % len(polygon)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


class _GoalEnd(Exception):
    """Raised inside an execute loop to end the goal with a terminal state."""

    def __init__(self, terminal_state, code, detail):
        super().__init__(detail)
        self.terminal_state = terminal_state
        self.code = code
        self.detail = detail


class MockRobotAgent(Node):
    """Mock robot agent serving the shared P1 interfaces."""

    def __init__(self, **kwargs):
        super().__init__('robot_agent', **kwargs)
        self.robot_id = validate_robot_id(self.declare_parameter('robot_id', 'robot_1').value)
        config_dir = self.declare_parameter('config_dir', '').value or None
        run_mode = self.declare_parameter('run_mode', 'development').value
        self.pose = [self.declare_parameter('initial_x', 0.0).value,
                     self.declare_parameter('initial_y', 0.0).value,
                     self.declare_parameter('initial_yaw', 0.0).value]
        self.linear_speed = self.declare_parameter('linear_speed', 0.15).value
        self.angular_speed = self.declare_parameter('angular_speed', 0.30).value
        self.localized = self.declare_parameter('localized', True).value
        self.fail_navigation_after_s = self.declare_parameter('fail_navigation_after_s', 0.0).value
        flat = list(self.declare_parameter('blocked_region', [0.0]).value)
        self.blocked_region = ([(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]
                               if len(flat) >= 6 else [])
        self.relocalize_succeeds = self.declare_parameter('relocalize_succeeds', True).value
        self.go_stale_after_s = self.declare_parameter('go_stale_after_s', 0.0).value
        self.publish_tf = self.declare_parameter('publish_tf', False).value

        contract = load_runtime_contract(config_dir)
        self.timeouts = contract['timeouts_s']
        rates = contract['rates_hz']
        self.start_poses = resolve_parameters(run_mode, config_dir).get(
            'localization.start_poses', {}) or {}

        self.lock = threading.RLock()
        self.velocity = (0.0, 0.0)
        self.competition = None
        self.last_state_time = None
        self.estop_generation = None
        self.estop_active = False
        self.active_goal = None
        self.abort_request = None
        self.relocalize_failed = False
        self.started = time.monotonic()

        group = ReentrantCallbackGroup()
        control_qos = qos_from_contract(contract, 'control_state')
        self.create_subscription(CompetitionState, '/competition/state', self._on_state,
                                 control_qos, callback_group=group)
        self.create_subscription(EmergencyStop, '/competition/estop', self._on_estop,
                                 control_qos, callback_group=group)
        self.state_pub = self.create_publisher(
            RobotState, 'state', qos_from_contract(contract, 'robot_state'))
        self.ack_pub = self.create_publisher(
            RobotPhaseAck, 'phase_ack', qos_from_contract(contract, 'robot_state'))
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.nav_server = ActionServer(
            self, ExecuteNavigationGoal, 'execute_navigation_goal',
            execute_callback=self._execute_navigation,
            goal_callback=self._accept_navigation,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.rescue_server = ActionServer(
            self, ExecuteRescueTask, 'execute_rescue_task',
            execute_callback=self._execute_rescue,
            goal_callback=self._accept_rescue,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.create_service(Relocalize, 'relocalize', self._relocalize, callback_group=group)

        self.create_timer(1.0 / rates['robot_state'], self._publish_state, callback_group=group)
        self.create_timer(1.0 / rates['robot_phase_ack_until_accepted'], self._publish_ack,
                          callback_group=group)
        self.get_logger().info(f'mock robot agent {self.robot_id} ready at {self.pose}')

    # ------------------------------------------------------------------ inputs
    def _on_state(self, msg):
        with self.lock:
            previous = self.competition
            self.competition = msg
            self.last_state_time = time.monotonic()
            changed = previous is None or previous.phase_sequence != msg.phase_sequence
            if changed and self.active_goal and msg.phase not in MOTION_PHASES:
                self.abort_request = (TaskReason.PHASE_CHANGED,
                                      f'phase changed to {msg.phase}')

    def _on_estop(self, msg):
        with self.lock:
            if self.estop_generation is not None and msg.generation <= self.estop_generation:
                return
            self.estop_generation = msg.generation
            self.estop_active = msg.active
            if msg.active and self.active_goal:
                self.abort_request = (TaskReason.SAFETY_STOP, f'emergency stop: {msg.reason}')

    def _edge_connected(self):
        return (self.last_state_time is not None and
                time.monotonic() - self.last_state_time <= self.timeouts['edge_connection'])

    def _motion_allowed(self):
        return (self.competition is not None and self.competition.phase in MOTION_PHASES and
                not self.estop_active and self._edge_connected())

    # ----------------------------------------------------------- goal checks
    def _common_rejection(self, run_id, map_id, map_revision, pose_frame):
        if self.active_goal is not None:
            return 'another goal is active; cancel it first'
        if self.competition is None:
            return 'no CompetitionState received'
        if run_id != self.competition.run_id:
            return 'run_id mismatch'
        if (map_id, map_revision) != (self.competition.map_id, self.competition.map_revision):
            return 'map_id/map_revision mismatch'
        if not self._motion_allowed():
            return 'motion not allowed in the current phase or safety state'
        if not self.localized:
            return 'robot is not localized'
        if pose_frame != 'map':
            return 'goal pose must be in the map frame'
        return None

    def _accept_navigation(self, goal):
        with self.lock:
            reason = self._common_rejection(goal.run_id, goal.map_id, goal.map_revision,
                                            goal.goal_pose.header.frame_id)
            if reason is None:
                self.active_goal = ('navigation', goal.request_id)
                self.abort_request = None
                return GoalResponse.ACCEPT
        self.get_logger().warn(f'rejected navigation goal: {reason}')
        return GoalResponse.REJECT

    def _accept_rescue(self, goal):
        with self.lock:
            reason = self._common_rejection(goal.run_id, goal.map_id, goal.map_revision,
                                            goal.rescue_pose.header.frame_id)
            if reason is None and goal.arrival_xy_tolerance_m <= 0.0:
                reason = 'arrival_xy_tolerance_m must be positive'
            if reason is None:
                self.active_goal = ('rescue', goal.task_id)
                self.abort_request = None
                return GoalResponse.ACCEPT
        self.get_logger().warn(f'rejected rescue goal: {reason}')
        return GoalResponse.REJECT

    # ----------------------------------------------------------------- motion
    def _step_towards(self, tx, ty, dt):
        """Advance the pose one step towards (tx, ty); return the remaining distance."""
        x, y, yaw = self.pose
        distance = math.hypot(tx - x, ty - y)
        if distance < 1e-3:
            self.velocity = (0.0, 0.0)
            return distance
        heading_error = normalize_angle(math.atan2(ty - y, tx - x) - yaw)
        if abs(heading_error) > 0.05:
            turn = math.copysign(min(abs(heading_error), self.angular_speed * dt), heading_error)
            self.pose[2] = normalize_angle(yaw + turn)
            self.velocity = (0.0, turn / dt)
            return distance
        step = min(distance, self.linear_speed * dt)
        self.pose[0] += step * math.cos(yaw)
        self.pose[1] += step * math.sin(yaw)
        self.velocity = (step / dt, 0.0)
        return distance - step

    def _turn_to(self, target_yaw, dt):
        error = normalize_angle(target_yaw - self.pose[2])
        turn = math.copysign(min(abs(error), self.angular_speed * dt), error)
        self.pose[2] = normalize_angle(self.pose[2] + turn)
        self.velocity = (0.0, turn / dt)
        return abs(error) - abs(turn)

    def _check_interrupts(self, goal_handle, start, cancel_code):
        if goal_handle.is_cancel_requested:
            raise _GoalEnd('cancelled', cancel_code, 'cancel requested')
        with self.lock:
            request = self.abort_request
        if request is not None:
            code, detail = request
            raise _GoalEnd('cancelled', code, detail)
        if 0.0 < self.fail_navigation_after_s <= time.monotonic() - start:
            raise _GoalEnd('failed', TaskReason.NAVIGATION_FAILED, 'injected navigation failure')

    def _finish(self, goal_handle, result, terminal_state, code, detail):
        self.velocity = (0.0, 0.0)
        result.reason = TaskReason(code=code, detail=detail)
        result.final_pose = self._pose_stamped()
        with self.lock:
            self.active_goal = None
            self.abort_request = None
        if goal_handle.is_cancel_requested:
            goal_handle.canceled(result)
        elif terminal_state in ('succeeded',):
            goal_handle.succeed(result)
        else:
            goal_handle.abort(result)
        return result

    # ------------------------------------------------------------- navigation
    def _execute_navigation(self, goal_handle):
        goal = goal_handle.request
        Result = ExecuteNavigationGoal.Result
        result = Result()
        tx, ty = goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y
        target_yaw = yaw_from_quaternion(goal.goal_pose.pose.orientation)
        xy_tol = goal.xy_tolerance_m if goal.xy_tolerance_m > 0.0 else 0.10
        yaw_tol = goal.yaw_tolerance_rad
        timeout = duration_seconds(goal.timeout) or self.timeouts['navigation_default']
        start = time.monotonic()
        last_feedback = 0.0
        dt = 1.0 / LOOP_HZ
        terminal = {'succeeded': Result.TERMINAL_SUCCEEDED, 'failed': Result.TERMINAL_FAILED,
                    'cancelled': Result.TERMINAL_CANCELLED, 'blocked': Result.TERMINAL_BLOCKED}
        try:
            while True:
                self._check_interrupts(goal_handle, start, TaskReason.USER_REQUEST)
                elapsed = time.monotonic() - start
                if self.blocked_region and point_in_polygon(tx, ty, self.blocked_region) \
                        and elapsed > 1.0:
                    raise _GoalEnd('blocked', TaskReason.NAVIGATION_FAILED,
                                   'blocked: goal inside blocked_region')
                if elapsed > timeout:
                    raise _GoalEnd('failed', TaskReason.NAVIGATION_TIMEOUT, 'navigation timeout')
                remaining = math.hypot(tx - self.pose[0], ty - self.pose[1])
                if self._motion_allowed():
                    if remaining > xy_tol:
                        remaining = self._step_towards(tx, ty, dt)
                    elif yaw_tol > 0.0 and \
                            abs(normalize_angle(target_yaw - self.pose[2])) > yaw_tol:
                        self._turn_to(target_yaw, dt)
                    else:
                        raise _GoalEnd('succeeded', TaskReason.NONE, 'goal reached')
                else:
                    self.velocity = (0.0, 0.0)
                if elapsed - last_feedback >= 0.5:
                    last_feedback = elapsed
                    feedback = ExecuteNavigationGoal.Feedback()
                    feedback.execution_state = ExecuteNavigationGoal.Feedback.STATE_NAVIGATING
                    feedback.distance_remaining_m = float(remaining)
                    feedback.elapsed = DurationMsg(sec=int(elapsed),
                                                   nanosec=int((elapsed % 1) * 1e9))
                    goal_handle.publish_feedback(feedback)
                time.sleep(dt)
        except _GoalEnd as end:
            result.terminal_state = terminal[end.terminal_state]
            result.success = end.terminal_state == 'succeeded'
            result.completed_at = self.get_clock().now().to_msg()
            return self._finish(goal_handle, result, end.terminal_state, end.code, end.detail)

    # ----------------------------------------------------------------- rescue
    def _execute_rescue(self, goal_handle):
        goal = goal_handle.request
        Result = ExecuteRescueTask.Result
        Feedback = ExecuteRescueTask.Feedback
        result = Result()
        tx, ty = goal.rescue_pose.pose.position.x, goal.rescue_pose.pose.position.y
        tolerance = goal.arrival_xy_tolerance_m
        hold_timeout = duration_seconds(goal.hold_timeout)
        start = time.monotonic()
        holding_since = None
        last_feedback = 0.0
        dt = 1.0 / LOOP_HZ
        terminal = {'failed': Result.TERMINAL_FAILED,
                    'cancelled': Result.TERMINAL_CANCELLED,
                    'blocked': Result.TERMINAL_BLOCKED,
                    'hold_timeout': Result.TERMINAL_HOLD_TIMEOUT}
        try:
            while True:
                # A coordinator cancel while motion is allowed releases the robot after the
                # rescue verifier's decision; otherwise the phase or e-stop caused it.
                cancel_code = (TaskReason.RELEASED_BY_COORDINATOR if self._motion_allowed()
                               else TaskReason.PHASE_CHANGED)
                if self.estop_active:
                    cancel_code = TaskReason.SAFETY_STOP
                self._check_interrupts(goal_handle, start, cancel_code)
                if self.blocked_region and point_in_polygon(tx, ty, self.blocked_region) \
                        and time.monotonic() - start > 1.0:
                    raise _GoalEnd('blocked', TaskReason.NAVIGATION_FAILED,
                                   'blocked: rescue pose inside blocked_region')
                remaining = math.hypot(tx - self.pose[0], ty - self.pose[1])
                if holding_since is None:
                    if remaining <= tolerance:
                        holding_since = time.monotonic()
                        self.velocity = (0.0, 0.0)
                    elif self._motion_allowed():
                        remaining = self._step_towards(tx, ty, dt)
                    else:
                        self.velocity = (0.0, 0.0)
                elif hold_timeout > 0.0 and time.monotonic() - holding_since >= hold_timeout:
                    raise _GoalEnd('hold_timeout', TaskReason.HOLD_TIMEOUT, 'hold timeout')
                elapsed = time.monotonic() - start
                if elapsed - last_feedback >= 0.5:
                    last_feedback = elapsed
                    feedback = Feedback()
                    feedback.execution_state = (Feedback.STATE_HOLDING if holding_since
                                                else Feedback.STATE_NAVIGATING)
                    feedback.distance_to_rescue_pose_m = float(remaining)
                    goal_handle.publish_feedback(feedback)
                time.sleep(dt)
        except _GoalEnd as end:
            result.terminal_state = terminal[end.terminal_state]
            result.ended_at = self.get_clock().now().to_msg()
            return self._finish(goal_handle, result, end.terminal_state, end.code, end.detail)

    # ------------------------------------------------------------- relocalize
    def _relocalize(self, request, response):
        def fail(code, message):
            response.success = False
            response.reason = TaskReason(code=code, detail=message)
            response.message = message
            return response

        with self.lock:
            if self.competition is None or request.run_id != self.competition.run_id:
                return fail(TaskReason.CONFIGURATION_INVALID, 'run_id mismatch')
            if (request.map_id, request.map_revision) != (self.competition.map_id,
                                                          self.competition.map_revision):
                return fail(TaskReason.CONFIGURATION_INVALID, 'map_id/map_revision mismatch')
        if not self.relocalize_succeeds:
            self.relocalize_failed = True
            return fail(TaskReason.RELOCALIZATION_FAILED, 'injected relocalization failure')
        if request.mode == Relocalize.Request.MODE_KNOWN_START_POSE:
            pose = self.start_poses.get(request.start_pose_id)
            if pose is None:
                return fail(TaskReason.CONFIGURATION_INVALID,
                            f'unknown start_pose_id {request.start_pose_id!r}')
            self.pose = [float(pose['x']), float(pose['y']), float(pose['yaw'])]
        elif request.mode == Relocalize.Request.MODE_POSE_HINT:
            hint = request.pose_hint
            if hint.header.frame_id != 'map':
                return fail(TaskReason.CONFIGURATION_INVALID, 'pose_hint must be in map')
            self.pose = [hint.pose.pose.position.x, hint.pose.pose.position.y,
                         yaw_from_quaternion(hint.pose.pose.orientation)]
        elif request.mode != Relocalize.Request.MODE_GLOBAL:
            return fail(TaskReason.CONFIGURATION_INVALID, f'unknown mode {request.mode}')
        # MODE_GLOBAL: the mock "finds" its current pose.
        self.localized = True
        self.relocalize_failed = False
        response.success = True
        response.estimated_pose.header.frame_id = 'map'
        response.estimated_pose.header.stamp = self.get_clock().now().to_msg()
        response.estimated_pose.pose.pose = self._pose_stamped().pose
        response.estimated_pose.pose.covariance[0] = 0.01
        response.estimated_pose.pose.covariance[7] = 0.01
        response.estimated_pose.pose.covariance[35] = 0.01
        response.reason = TaskReason(code=TaskReason.NONE, detail='')
        response.message = 'relocalized (mock)'
        return response

    # ----------------------------------------------------------------- output
    def _pose_stamped(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.pose[0], self.pose[1]
        msg.pose.orientation.z = math.sin(self.pose[2] / 2.0)
        msg.pose.orientation.w = math.cos(self.pose[2] / 2.0)
        return msg

    def _mode(self):
        if self.estop_active:
            return RobotState.MODE_ESTOP
        if not self._motion_allowed():
            return RobotState.MODE_SAFE
        if self.active_goal is None:
            return RobotState.MODE_STANDBY
        if self.active_goal[0] == 'rescue' and self.velocity == (0.0, 0.0):
            return RobotState.MODE_HOLDING
        if self.active_goal[0] == 'navigation' and self.competition.phase == \
                CompetitionState.PHASE_TASK1:
            return RobotState.MODE_EXPLORING
        return RobotState.MODE_NAVIGATING

    def _faults(self):
        faults = []
        if self.estop_active:
            faults.append(SystemFault(source=self.get_name(), code=SystemFault.CODE_ESTOP_ACTIVE,
                                      severity=SystemFault.SEVERITY_WARN, detail='estop'))
        if self.last_state_time is not None and not self._edge_connected():
            faults.append(SystemFault(source=self.get_name(),
                                      code=SystemFault.CODE_EDGE_CONNECTION_LOST,
                                      severity=SystemFault.SEVERITY_ERROR,
                                      detail='no CompetitionState heartbeat'))
        if self.relocalize_failed:
            faults.append(SystemFault(source=self.get_name(),
                                      code=SystemFault.CODE_RELOCALIZATION_FAILED,
                                      severity=SystemFault.SEVERITY_ERROR,
                                      detail='last relocalize failed'))
        return faults

    def _publish_state(self):
        if 0.0 < self.go_stale_after_s <= time.monotonic() - self.started:
            return
        with self.lock:
            msg = RobotState()
            pose = self._pose_stamped()
            msg.header = pose.header
            msg.run_id = self.competition.run_id if self.competition else ''
            msg.robot_id = self.robot_id
            msg.mode = self._mode()
            msg.pose.pose = pose.pose
            msg.pose.covariance[0] = msg.pose.covariance[7] = msg.pose.covariance[35] = 0.01
            msg.velocity.twist.linear.x = self.velocity[0]
            msg.velocity.twist.angular.z = self.velocity[1]
            msg.localized = self.localized
            msg.navigation_available = True
            msg.battery_percentage = float('nan')
            msg.active_task_id = self.active_goal[1] if self.active_goal else ''
            msg.faults = self._faults()
        self.state_pub.publish(msg)
        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = pose.header
            transform.child_frame_id = frame(self.robot_id, 'base_footprint')
            transform.transform.translation.x = pose.pose.position.x
            transform.transform.translation.y = pose.pose.position.y
            transform.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def _publish_ack(self):
        with self.lock:
            if self.competition is None:
                return
            msg = RobotPhaseAck()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.run_id = self.competition.run_id
            msg.robot_id = self.robot_id
            msg.phase_sequence = self.competition.phase_sequence
            msg.phase = self.competition.phase
            stopped = self.velocity == (0.0, 0.0)
            motion_phase = self.competition.phase in MOTION_PHASES
            msg.phase_received = True
            msg.actions_cancelled = motion_phase or self.active_goal is None
            msg.zero_velocity_commanded = motion_phase or stopped
            msg.safe = stopped
            msg.localized = self.localized
            msg.ready_for_phase = self.localized and (motion_phase or self.active_goal is None)
            msg.faults = self._faults()
        self.ack_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRobotAgent()
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
