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

"""Behavioural tests for mock_robot_agent (see the package README)."""

import math
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.action import ExecuteNavigationGoal, ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState,
    EmergencyStop,
    RobotPhaseAck,
    RobotState,
    TaskReason,
)
from edgeai_robot_interfaces.srv import Relocalize
from edgeai_system_tests.mocks.mock_robot_agent import MockRobotAgent
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

RUN_ID = 'run-test'
MAP_ID = 'map-test'


def _config_dir():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'config' / 'competition_parameters.yaml').is_file():
            return str(parent / 'config')
    pytest.skip('repository config directory not found')


def wait_for(predicate, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class Harness:
    """Edge-side stand-in: publishes CompetitionState/EmergencyStop and calls the mock."""

    def __init__(self, **mock_params):
        config_dir = _config_dir()
        contract = load_runtime_contract(config_dir)
        params = {'robot_id': 'robot_1', 'config_dir': config_dir, 'linear_speed': 1.0,
                  'angular_speed': 3.0}
        params.update(mock_params)
        self.mock = MockRobotAgent(
            namespace='robot_1',
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.node = Node('edge_harness')
        qos = qos_from_contract(contract, 'control_state')
        self.state_pub = self.node.create_publisher(CompetitionState, '/competition/state', qos)
        self.estop_pub = self.node.create_publisher(EmergencyStop, '/competition/estop', qos)
        self.states, self.acks = [], []
        self.node.create_subscription(RobotState, '/robot_1/state', self.states.append,
                                      qos_from_contract(contract, 'robot_state'))
        self.node.create_subscription(RobotPhaseAck, '/robot_1/phase_ack', self.acks.append,
                                      qos_from_contract(contract, 'robot_state'))
        self.nav = ActionClient(self.node, ExecuteNavigationGoal,
                                '/robot_1/execute_navigation_goal')
        self.rescue = ActionClient(self.node, ExecuteRescueTask, '/robot_1/execute_rescue_task')
        self.relocalize = self.node.create_client(Relocalize, '/robot_1/relocalize')
        self.phase = CompetitionState.PHASE_TASK2A
        self.sequence = 1
        self.heartbeat = True
        self.node.create_timer(0.2, self._publish_state)
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.mock)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.nav.wait_for_server(timeout_sec=5.0)
        assert self.rescue.wait_for_server(timeout_sec=5.0)
        assert wait_for(lambda: self.mock.competition is not None)

    def _publish_state(self):
        if self.heartbeat:
            self.state_pub.publish(CompetitionState(
                run_id=RUN_ID, phase_sequence=self.sequence, map_id=MAP_ID, map_revision=1,
                phase=self.phase))

    def set_phase(self, phase):
        self.phase = phase
        self.sequence += 1
        self._publish_state()

    def estop(self, generation, active):
        self.estop_pub.publish(EmergencyStop(run_id=RUN_ID, generation=generation,
                                             active=active, source='test'))

    def _send(self, client, goal):
        future = client.send_goal_async(goal)
        assert wait_for(future.done)
        return future.result()

    def send_navigation(self, x, y, **fields):
        goal = ExecuteNavigationGoal.Goal(run_id=RUN_ID, request_id='nav-1', map_id=MAP_ID,
                                          map_revision=1, xy_tolerance_m=0.05)
        goal.goal_pose.header.frame_id = 'map'
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = x, y
        goal.goal_pose.pose.orientation.w = 1.0
        for key, value in fields.items():
            setattr(goal, key, value)
        return self._send(self.nav, goal)

    def send_rescue(self, x, y, hold_timeout_s=0):
        goal = ExecuteRescueTask.Goal(run_id=RUN_ID, task_id='task-1', target_id='T1',
                                      map_id=MAP_ID, map_revision=1,
                                      arrival_xy_tolerance_m=0.08)
        goal.rescue_pose.header.frame_id = 'map'
        goal.rescue_pose.pose.position.x, goal.rescue_pose.pose.position.y = x, y
        goal.hold_timeout.sec = hold_timeout_s
        return self._send(self.rescue, goal)

    @staticmethod
    def result(goal_handle, timeout=15.0):
        future = goal_handle.get_result_async()
        assert wait_for(future.done, timeout)
        return future.result()

    def close(self):
        self.executor.shutdown()
        self.mock.destroy_node()
        self.node.destroy_node()


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


def test_navigation_succeeds(harness_factory):
    h = harness_factory()
    handle = h.send_navigation(0.5, 0.0)
    assert handle.accepted
    outcome = h.result(handle)
    assert outcome.status == GoalStatus.STATUS_SUCCEEDED
    assert outcome.result.terminal_state == ExecuteNavigationGoal.Result.TERMINAL_SUCCEEDED
    assert outcome.result.success
    assert math.hypot(outcome.result.final_pose.pose.position.x - 0.5,
                      outcome.result.final_pose.pose.position.y) <= 0.05 + 1e-6


def test_goal_rejected_outside_motion_phase(harness_factory):
    h = harness_factory()
    h.set_phase(CompetitionState.PHASE_PREPARATION)
    assert wait_for(lambda: h.mock.competition.phase == CompetitionState.PHASE_PREPARATION)
    assert not h.send_navigation(0.5, 0.0).accepted


def test_second_goal_rejected_while_one_is_active(harness_factory):
    h = harness_factory(linear_speed=0.05)
    assert h.send_navigation(2.0, 0.0).accepted
    assert not h.send_navigation(1.0, 0.0).accepted


def test_injected_navigation_failure(harness_factory):
    h = harness_factory(linear_speed=0.05, fail_navigation_after_s=0.3)
    outcome = h.result(h.send_navigation(2.0, 0.0))
    assert outcome.result.terminal_state == ExecuteNavigationGoal.Result.TERMINAL_FAILED
    assert outcome.result.reason.code == TaskReason.NAVIGATION_FAILED


def test_cancel_navigation(harness_factory):
    h = harness_factory(linear_speed=0.05)
    handle = h.send_navigation(2.0, 0.0)
    time.sleep(0.3)
    cancel = handle.cancel_goal_async()
    assert wait_for(cancel.done)
    outcome = h.result(handle)
    assert outcome.status == GoalStatus.STATUS_CANCELED
    assert outcome.result.terminal_state == ExecuteNavigationGoal.Result.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.USER_REQUEST


def test_estop_interrupts_and_stale_generation_is_ignored(harness_factory):
    h = harness_factory(linear_speed=0.05)
    handle = h.send_navigation(2.0, 0.0)
    time.sleep(0.2)
    h.estop(generation=2, active=True)
    outcome = h.result(handle)
    assert outcome.result.terminal_state == ExecuteNavigationGoal.Result.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.SAFETY_STOP
    h.estop(generation=1, active=False)  # older generation: must be ignored
    time.sleep(0.3)
    assert h.mock.estop_active
    assert wait_for(lambda: h.states and h.states[-1].mode == RobotState.MODE_ESTOP)
    h.estop(generation=3, active=False)
    assert wait_for(lambda: not h.mock.estop_active)


def test_rescue_holds_until_released(harness_factory):
    h = harness_factory()
    handle = h.send_rescue(0.3, 0.0)
    assert handle.accepted
    assert wait_for(lambda: h.states and h.states[-1].mode == RobotState.MODE_HOLDING)
    time.sleep(0.5)
    assert h.mock.active_goal is not None  # the robot never completes a rescue by itself
    cancel = handle.cancel_goal_async()
    assert wait_for(cancel.done)
    outcome = h.result(handle)
    assert outcome.result.terminal_state == ExecuteRescueTask.Result.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.RELEASED_BY_COORDINATOR


def test_rescue_hold_timeout(harness_factory):
    h = harness_factory()
    outcome = h.result(h.send_rescue(0.3, 0.0, hold_timeout_s=1))
    assert outcome.result.terminal_state == ExecuteRescueTask.Result.TERMINAL_HOLD_TIMEOUT
    assert outcome.result.reason.code == TaskReason.HOLD_TIMEOUT


def test_phase_change_cancels_goal_and_is_acknowledged(harness_factory):
    h = harness_factory(linear_speed=0.05)
    handle = h.send_navigation(2.0, 0.0)
    time.sleep(0.2)
    h.set_phase(CompetitionState.PHASE_FROZEN)
    outcome = h.result(handle)
    assert outcome.result.terminal_state == ExecuteNavigationGoal.Result.TERMINAL_CANCELLED
    assert outcome.result.reason.code == TaskReason.PHASE_CHANGED
    assert wait_for(lambda: any(a.phase_sequence == h.sequence and a.safe and
                                a.phase == CompetitionState.PHASE_FROZEN for a in h.acks))


def test_lost_edge_heartbeat_reports_safe_mode(harness_factory):
    h = harness_factory()
    h.heartbeat = False
    assert wait_for(lambda: h.states and h.states[-1].mode == RobotState.MODE_SAFE, timeout=5)


def test_relocalize_known_start_pose(harness_factory):
    h = harness_factory()
    assert h.relocalize.wait_for_service(timeout_sec=5.0)
    request = Relocalize.Request(run_id=RUN_ID, map_id=MAP_ID, map_revision=1,
                                 mode=Relocalize.Request.MODE_KNOWN_START_POSE,
                                 start_pose_id='task1_start')
    future = h.relocalize.call_async(request)
    assert wait_for(future.done)
    response = future.result()
    assert response.success, response.message
    assert response.estimated_pose.pose.pose.position.x == pytest.approx(-1.5)
    assert response.estimated_pose.pose.pose.position.y == pytest.approx(-1.5)


def test_relocalize_failure_is_reported(harness_factory):
    h = harness_factory(relocalize_succeeds=False)
    assert h.relocalize.wait_for_service(timeout_sec=5.0)
    future = h.relocalize.call_async(Relocalize.Request(
        run_id=RUN_ID, map_id=MAP_ID, map_revision=1, mode=Relocalize.Request.MODE_GLOBAL))
    assert wait_for(future.done)
    assert not future.result().success
    assert future.result().reason.code == TaskReason.RELOCALIZATION_FAILED
