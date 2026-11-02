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

"""Tests for robot_agent with a fake Nav2, TF, odometry and AMCL (P1 plan 2-10 .. 2-14)."""

import math
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_bringup.robot_agent import (
    alignment_turn,
    covariance_ok,
    map_nav2_failure,
    ModeInputs,
    RobotAgent,
    select_mode,
    straight_line_move,
    within_tolerance,
)
from edgeai_robot_interfaces.action import ExecuteNavigationGoal, ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState,
    EmergencyStop,
    RobotPhaseAck,
    RobotState,
    SystemFault,
    TaskReason,
)
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

Result = ExecuteNavigationGoal.Result
RescueResult = ExecuteRescueTask.Result
RUN_ID, MAP_ID = 'run-test', 'map-test'


# ---------------------------------------------------------------- pure logic

def test_mode_priority():
    base = {'estop_active': False, 'motion_allowed': True, 'fatal_fault': False,
            'active_kind': None, 'holding': False, 'exploring': False}

    def mode(**changes):
        return select_mode(ModeInputs(**{**base, **changes}))

    assert mode() == RobotState.MODE_STANDBY
    assert mode(active_kind='navigation') == RobotState.MODE_NAVIGATING
    assert mode(active_kind='navigation', exploring=True) == RobotState.MODE_EXPLORING
    assert mode(active_kind='rescue', holding=True) == RobotState.MODE_HOLDING
    assert mode(active_kind='navigation', fatal_fault=True) == RobotState.MODE_ERROR
    assert mode(active_kind='navigation', motion_allowed=False) == RobotState.MODE_SAFE
    assert mode(estop_active=True, motion_allowed=False) == RobotState.MODE_ESTOP


def test_nav2_failure_mapping():
    assert map_nav2_failure(6, 105)[:2] == (Result.TERMINAL_BLOCKED,
                                            TaskReason.NAVIGATION_FAILED)
    assert map_nav2_failure(6, 104)[0] == Result.TERMINAL_BLOCKED
    assert map_nav2_failure(6, 208)[:2] == (Result.TERMINAL_FAILED,
                                            TaskReason.NAVIGATION_FAILED)
    assert map_nav2_failure(6, 207)[1] == TaskReason.NAVIGATION_TIMEOUT


def test_tolerance_and_covariance_checks():
    assert within_tolerance((1.0, 1.0, 0.0), (1.05, 1.0, 3.0), 0.10, 0.0)
    assert not within_tolerance((1.0, 1.0, 0.0), (1.05, 1.0, 3.0), 0.10, 0.25)
    assert not within_tolerance((1.0, 1.0, 0.0), (1.2, 1.0, 0.0), 0.10, 0.0)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.01
    cov[35] = 0.01
    assert covariance_ok(cov, 0.15, 0.15)
    cov[0] = 0.04
    assert not covariance_ok(cov, 0.15, 0.15)


def test_straight_line_move():
    # Pushed forward along the heading: back up; pushed backward: drive forward.
    assert straight_line_move((1.3, 0.0, 0.0), (1.0, 0.0), 0.6, 0.04) == pytest.approx(-0.3)
    assert straight_line_move((0.0, 0.7, -math.pi / 2), (0.0, 1.0), 0.6, 0.04) == \
        pytest.approx(-0.3)
    assert straight_line_move((0.7, 0.0, 0.0), (1.0, 0.0), 0.6, 0.04) == pytest.approx(0.3)
    assert straight_line_move((1.0, 0.3, 0.0), (1.0, 0.0), 0.6, 0.04) is None   # sideways
    assert straight_line_move((2.0, 0.0, 0.0), (1.0, 0.0), 0.6, 0.04) is None   # too far


def test_alignment_turn():
    # Slot 0.55 m behind and 0.1 m to the left (+y) of a robot facing +x: turn so the
    # rear points at it (heading away from the slot), here a small clockwise turn.
    turn = alignment_turn((1.70, 4.56, 0.0), (1.15, 4.46), 0.6, 0.6)
    assert turn == pytest.approx(math.atan2(0.1, 0.55))
    assert straight_line_move((1.70, 4.56, turn), (1.15, 4.46), 0.6, 0.04) == \
        pytest.approx(-math.hypot(0.55, 0.1))
    # Slot 0.8 m ahead and 0.1 m to the right: a small clockwise turn, then forward.
    turn = alignment_turn((0.0, 0.0, 0.0), (0.8, -0.1), 0.85, 0.6)
    assert turn == pytest.approx(math.atan2(-0.1, 0.8))
    assert straight_line_move((0.0, 0.0, turn), (0.8, -0.1), 0.85, 0.04) == \
        pytest.approx(math.hypot(0.8, 0.1))
    assert alignment_turn((2.0, 0.0, 0.0), (1.0, 0.0), 0.6, 0.6) is None   # too far
    assert alignment_turn((1.0, 0.4, 0.0), (0.7, 0.0), 0.6, 0.6) is None   # big turn


# ---------------------------------------------------------------- fake robot

def _config_dir():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'config' / 'interface_runtime.yaml').is_file():
            return str(parent / 'config')
    pytest.skip('repository config directory not found')


class FakeRobot(Node):
    """Nav2 navigate_to_pose, lifecycle is_active, TF, odometry, AMCL pose and scan."""

    def __init__(self):
        super().__init__('fake_robot', namespace='robot_1')
        group = ReentrantCallbackGroup()
        self.pose = [0.0, 0.0, 0.0]
        self.behaviour = 'succeed'      # succeed | fail | hang
        self.error_code = 0
        self.arrival_offset = 0.0
        self.moving = False
        self.force_moving = False       # odometry reports motion regardless of Nav2
        self.nav_active = True
        self.scan_enabled = True
        self.covariance = [0.0] * 36
        self.covariance[0] = self.covariance[7] = self.covariance[35] = 0.01
        self.nav_goals = 0
        self.straight_goals = 0
        self.cancelled = 0
        self.tf = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        # Same QoS as Nav2 AMCL's amcl_pose publisher.
        self.amcl_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'amcl_pose',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.create_service(Trigger, 'lifecycle_manager_navigation/is_active',
                            lambda _req, res: self._is_active(res), callback_group=group)
        ActionServer(self, NavigateToPose, 'navigate_to_pose', execute_callback=self._navigate,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        for action, name, sign in ((BackUp, 'backup', -1.0),
                                   (DriveOnHeading, 'drive_on_heading', 1.0)):
            ActionServer(self, action, name,
                         execute_callback=lambda h, a=action, s=sign: self._straight(h, a, s),
                         cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.create_timer(0.05, self._publish, callback_group=group)

    def _is_active(self, response):
        response.success = self.nav_active
        return response

    def _navigate(self, goal_handle):
        self.nav_goals += 1
        target = goal_handle.request.pose.pose.position
        self.moving = True
        end = time.monotonic() + (60.0 if self.behaviour == 'hang' else 0.4)
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                self.cancelled += 1
                self.moving = False
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        self.moving = False
        result = NavigateToPose.Result()
        if self.behaviour == 'fail':
            result.error_code = self.error_code
            goal_handle.abort()
            return result
        self.pose = [target.x + self.arrival_offset, target.y, 0.0]
        goal_handle.succeed()
        return result

    def _straight(self, goal_handle, action, sign):
        self.straight_goals += 1
        distance = sign * goal_handle.request.target.x
        self.moving = True
        time.sleep(0.2)
        self.moving = False
        x, y, yaw = self.pose
        self.pose = [x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw]
        goal_handle.succeed()
        return action.Result()

    def _publish(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp, t.header.frame_id = now, 'map'
        t.child_frame_id = 'robot_1/base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.pose[0], self.pose[1]
        t.transform.rotation.w = 1.0
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = now
        odom.twist.twist.linear.x = 0.1 if (self.moving or self.force_moving) else 0.0
        self.odom_pub.publish(odom)
        amcl = PoseWithCovarianceStamped()
        amcl.header.stamp, amcl.header.frame_id = now, 'map'
        amcl.pose.covariance = self.covariance
        self.amcl_pub.publish(amcl)
        if self.scan_enabled:
            self.scan_pub.publish(LaserScan())


class Harness:

    def __init__(self, **params):
        config_dir = _config_dir()
        values = {'robot_id': 'robot_1', 'config_dir': config_dir}
        values.update(params)
        self.agent = RobotAgent(namespace='robot_1', parameter_overrides=[
            Parameter(k, value=v) for k, v in values.items()])
        self.robot = FakeRobot()
        self.edge = Node('edge_harness')
        qos = qos_from_contract(load_runtime_contract(config_dir), 'control_state')
        self.state_pub = self.edge.create_publisher(CompetitionState, '/competition/state', qos)
        self.estop_pub = self.edge.create_publisher(EmergencyStop, '/competition/estop', qos)
        self.states, self.acks = [], []
        self.edge.create_subscription(RobotState, '/robot_1/state', self.states.append, 10)
        self.edge.create_subscription(RobotPhaseAck, '/robot_1/phase_ack', self.acks.append, 10)
        self.client = ActionClient(self.edge, ExecuteNavigationGoal,
                                   '/robot_1/execute_navigation_goal')
        self.rescue_client = ActionClient(self.edge, ExecuteRescueTask,
                                          '/robot_1/execute_rescue_task')
        self.rescue_feedback = []
        self.phase, self.sequence = CompetitionState.PHASE_TASK2A, 1
        self.edge.create_timer(0.2, self.heartbeat)
        self.executor = MultiThreadedExecutor(num_threads=8)
        for node in (self.agent, self.robot, self.edge):
            self.executor.add_node(node)
        threading.Thread(target=self.executor.spin, daemon=True).start()
        assert self.client.wait_for_server(timeout_sec=5.0)
        assert self.rescue_client.wait_for_server(timeout_sec=5.0)
        assert wait_for(lambda: self.states and self.states[-1].localized and
                        self.states[-1].navigation_available, 10.0), 'agent never ready'

    def heartbeat(self):
        self.state_pub.publish(CompetitionState(run_id=RUN_ID, map_id=MAP_ID, map_revision=1,
                                                phase=self.phase,
                                                phase_sequence=self.sequence))

    def set_phase(self, phase):
        self.phase, self.sequence = phase, self.sequence + 1
        self.heartbeat()

    def send(self, x, y, **fields):
        goal = ExecuteNavigationGoal.Goal(run_id=RUN_ID, request_id='nav-1', map_id=MAP_ID,
                                          map_revision=1, xy_tolerance_m=0.10)
        goal.goal_pose.header.frame_id = 'map'
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = x, y
        goal.goal_pose.pose.orientation.w = 1.0
        for key, value in fields.items():
            setattr(goal, key, value)
        future = self.client.send_goal_async(goal)
        assert wait_for(future.done, 5.0)
        return future.result()

    def send_rescue(self, x, y, tolerance=0.08, hold_timeout_s=0):
        goal = ExecuteRescueTask.Goal(run_id=RUN_ID, task_id='task-1', target_id='T1',
                                      map_id=MAP_ID, map_revision=1,
                                      target_type=ExecuteRescueTask.Goal.TYPE_NORMAL,
                                      arrival_xy_tolerance_m=tolerance,
                                      hold_timeout=Duration(sec=hold_timeout_s))
        goal.rescue_pose.header.frame_id = 'map'
        goal.rescue_pose.pose.position.x, goal.rescue_pose.pose.position.y = x, y
        goal.rescue_pose.pose.orientation.w = 1.0
        future = self.rescue_client.send_goal_async(
            goal, feedback_callback=lambda m: self.rescue_feedback.append(m.feedback))
        assert wait_for(future.done, 5.0)
        return future.result()

    @staticmethod
    def result(handle, timeout=15.0):
        future = handle.get_result_async()
        assert wait_for(future.done, timeout)
        return future.result()

    def close(self):
        self.executor.shutdown()
        for node in (self.agent, self.robot, self.edge):
            node.destroy_node()


def wait_for(predicate, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def harness_factory():
    rclpy.init()
    created = []

    def make(**params):
        harness = Harness(**params)
        created.append(harness)
        return harness

    yield make
    for harness in created:
        harness.close()
    rclpy.shutdown()


# --------------------------------------------------------------- behaviour

def test_state_reports_pose_and_standby(harness_factory):
    h = harness_factory()
    state = h.states[-1]
    assert state.header.frame_id == 'map' and state.robot_id == 'robot_1'
    assert state.run_id == RUN_ID and state.mode == RobotState.MODE_STANDBY
    assert math.isnan(state.battery_percentage)


def test_navigation_succeeds_and_reports_navigating(harness_factory):
    h = harness_factory()
    handle = h.send(1.0, 0.5)
    assert handle.accepted
    assert wait_for(lambda: h.states[-1].mode == RobotState.MODE_NAVIGATING and
                    h.states[-1].active_task_id == 'nav-1', 3.0)
    outcome = h.result(handle)
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED
    assert outcome.result.terminal_state == Result.TERMINAL_SUCCEEDED and outcome.result.success
    assert outcome.result.final_pose.pose.position.x == pytest.approx(1.0, abs=0.01)
    assert wait_for(lambda: h.states[-1].mode == RobotState.MODE_STANDBY, 2.0)


def test_exploration_purpose_reports_exploring(harness_factory):
    h = harness_factory()
    h.robot.behaviour = 'hang'
    handle = h.send(1.0, 0.0, purpose=ExecuteNavigationGoal.Goal.PURPOSE_EXPLORATION)
    assert wait_for(lambda: h.states[-1].mode == RobotState.MODE_EXPLORING, 3.0)
    handle.cancel_goal_async()
    h.result(handle)


def test_rejections(harness_factory):
    h = harness_factory()
    assert not h.send(1.0, 0.0, run_id='other-run').accepted
    assert not h.send(1.0, 0.0, map_revision=2).accepted
    h.robot.covariance = [0.0] * 36
    h.robot.covariance[0] = h.robot.covariance[7] = 0.5   # std 0.7 m: not localized
    assert wait_for(lambda: not h.states[-1].localized, 3.0)
    assert not h.send(1.0, 0.0).accepted
    assert any(f.code == SystemFault.CODE_LOCALIZATION_LOST for f in h.states[-1].faults)


def test_rejected_when_navigation_inactive_or_phase_frozen(harness_factory):
    h = harness_factory()
    h.robot.nav_active = False
    assert wait_for(lambda: not h.states[-1].navigation_available, 3.0)
    assert not h.send(1.0, 0.0).accepted
    h.robot.nav_active = True
    assert wait_for(lambda: h.states[-1].navigation_available, 3.0)
    h.set_phase(CompetitionState.PHASE_FROZEN)
    assert wait_for(lambda: h.states[-1].mode == RobotState.MODE_SAFE, 2.0)
    assert not h.send(1.0, 0.0).accepted


@pytest.mark.parametrize('error_code, terminal, reason', [
    (105, Result.TERMINAL_BLOCKED, TaskReason.NAVIGATION_FAILED),
    (208, Result.TERMINAL_FAILED, TaskReason.NAVIGATION_FAILED),
])
def test_nav2_failures_are_mapped(harness_factory, error_code, terminal, reason):
    h = harness_factory()
    h.robot.behaviour, h.robot.error_code = 'fail', error_code
    outcome = h.result(h.send(1.0, 0.0))
    assert outcome.result.terminal_state == terminal
    assert outcome.result.reason.code == reason


def test_precise_approach_then_fail_outside_tolerance(harness_factory):
    h = harness_factory()
    h.robot.arrival_offset = 0.3
    outcome = h.result(h.send(1.0, 0.0))
    assert h.robot.nav_goals == 2
    assert outcome.result.terminal_state == Result.TERMINAL_FAILED
    assert 'tolerance' in outcome.result.reason.detail


def test_cancel_stops_nav2(harness_factory):
    h = harness_factory()
    h.robot.behaviour = 'hang'
    handle = h.send(1.0, 0.0)
    time.sleep(0.5)
    handle.cancel_goal_async()
    outcome = h.result(handle)
    assert outcome.status == GoalStatus.STATUS_CANCELED
    assert outcome.result.terminal_state == Result.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.USER_REQUEST
    assert h.robot.cancelled == 1


def test_phase_change_cancels_and_acknowledges(harness_factory):
    h = harness_factory()
    h.robot.behaviour = 'hang'
    handle = h.send(1.0, 0.0)
    time.sleep(0.5)
    h.set_phase(CompetitionState.PHASE_FROZEN)
    outcome = h.result(handle)
    assert outcome.result.terminal_state == Result.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.PHASE_CHANGED
    assert wait_for(lambda: any(a.phase_sequence == h.sequence and a.safe and
                                a.actions_cancelled and a.zero_velocity_commanded
                                for a in h.acks), 3.0)


def test_estop_cancels_and_old_generation_is_ignored(harness_factory):
    h = harness_factory()
    h.robot.behaviour = 'hang'
    handle = h.send(1.0, 0.0)
    time.sleep(0.5)
    h.estop_pub.publish(EmergencyStop(run_id=RUN_ID, generation=3, active=True))
    outcome = h.result(handle)
    assert outcome.result.reason.code == TaskReason.SAFETY_STOP
    h.estop_pub.publish(EmergencyStop(run_id=RUN_ID, generation=2, active=False))
    time.sleep(0.4)
    assert h.states[-1].mode == RobotState.MODE_ESTOP


def test_navigation_timeout(harness_factory):
    h = harness_factory()
    h.robot.behaviour = 'hang'
    goal = h.send(1.0, 0.0, timeout=Duration(sec=1))
    outcome = h.result(goal)
    assert outcome.result.terminal_state == Result.TERMINAL_FAILED
    assert outcome.result.reason.code == TaskReason.NAVIGATION_TIMEOUT


def test_no_standstill_raises_command_timeout_fault(harness_factory):
    h = harness_factory()
    h.robot.behaviour = 'hang'
    handle = h.send(1.0, 0.0)
    time.sleep(0.5)
    # Odometry keeps reporting motion after the cancel: the agent must still return a
    # result, but report that the robot did not stop.
    h.robot.force_moving = True
    handle.cancel_goal_async()
    outcome = h.result(handle, timeout=10.0)
    assert outcome.result.terminal_state == Result.TERMINAL_CANCELLED
    assert wait_for(lambda: any(f.code == SystemFault.CODE_COMMAND_TIMEOUT
                                for f in h.states[-1].faults), 2.0)
    h.robot.force_moving = False


def holding(h):
    return (h.states[-1].mode == RobotState.MODE_HOLDING and h.rescue_feedback and
            h.rescue_feedback[-1].execution_state == ExecuteRescueTask.Feedback.STATE_HOLDING)


def test_rescue_reaches_holds_and_is_released(harness_factory):
    h = harness_factory()
    handle = h.send_rescue(1.0, 0.5)
    assert handle.accepted
    assert wait_for(lambda: holding(h), 5.0)
    assert h.states[-1].active_task_id == 'task-1'
    time.sleep(1.0)
    assert holding(h)          # the robot never completes a rescue by itself
    handle.cancel_goal_async()
    outcome = h.result(handle)
    assert outcome.status == GoalStatus.STATUS_CANCELED
    assert outcome.result.terminal_state == RescueResult.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.RELEASED_BY_COORDINATOR
    assert wait_for(lambda: h.states[-1].mode == RobotState.MODE_STANDBY, 2.0)


def test_rescue_hold_timeout(harness_factory):
    h = harness_factory()
    outcome = h.result(h.send_rescue(1.0, 0.0, hold_timeout_s=1))
    assert outcome.result.terminal_state == RescueResult.TERMINAL_HOLD_TIMEOUT
    assert outcome.result.reason.code == TaskReason.HOLD_TIMEOUT


def test_rescue_reapproaches_when_pushed_away(harness_factory):
    h = harness_factory()
    handle = h.send_rescue(1.0, 0.0)
    assert wait_for(lambda: holding(h), 5.0)
    nav_goals = h.robot.nav_goals
    h.robot.pose = [1.3, 0.0, 0.0]            # pushed 0.3 m forward: backs up, no turn
    assert wait_for(lambda: h.robot.straight_goals == 1, 3.0)
    assert wait_for(lambda: holding(h) and h.robot.pose[0] == pytest.approx(1.0), 5.0)
    assert h.robot.nav_goals == nav_goals
    h.robot.pose = [1.0, 0.3, 0.0]            # pushed sideways: Nav2 re-approach
    assert wait_for(lambda: h.robot.nav_goals > nav_goals, 3.0)
    assert wait_for(lambda: holding(h) and h.robot.pose[1] == pytest.approx(0.0), 5.0)
    assert h.robot.straight_goals == 1
    handle.cancel_goal_async()
    h.result(handle)


def test_rescue_phase_change_while_holding(harness_factory):
    h = harness_factory()
    handle = h.send_rescue(1.0, 0.0)
    assert wait_for(lambda: holding(h), 5.0)
    h.set_phase(CompetitionState.PHASE_FROZEN)
    outcome = h.result(handle)
    assert outcome.result.terminal_state == RescueResult.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.PHASE_CHANGED


def test_rescue_rejects_non_positive_tolerance(harness_factory):
    h = harness_factory()
    assert not h.send_rescue(1.0, 0.0, tolerance=0.0).accepted


def test_rescue_approach_failure_and_tolerance(harness_factory):
    h = harness_factory()
    h.robot.behaviour, h.robot.error_code = 'fail', 105
    outcome = h.result(h.send_rescue(1.0, 0.0))
    assert outcome.result.terminal_state == RescueResult.TERMINAL_BLOCKED
    h.robot.behaviour, h.robot.arrival_offset = 'succeed', 0.3
    outcome = h.result(h.send_rescue(1.0, 0.0))
    assert outcome.result.terminal_state == RescueResult.TERMINAL_FAILED
    assert 'arrival tolerance' in outcome.result.reason.detail


def test_missing_scan_reports_sensor_fault(harness_factory):
    h = harness_factory()
    h.robot.scan_enabled = False
    assert wait_for(lambda: any(f.code == SystemFault.CODE_SENSOR_UNAVAILABLE
                                for f in h.states[-1].faults), 3.0)


def test_disabling_competition_state_requires_development():
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            RobotAgent(parameter_overrides=[
                Parameter('run_mode', value='rehearsal'),
                Parameter('require_competition_state', value=False),
                Parameter('config_dir', value=_config_dir())])
    finally:
        rclpy.shutdown()
