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

"""Tests for the safety_watchdog gate (ADR-0003)."""

from pathlib import Path
import threading
import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_bringup.safety_watchdog import decide, GateInputs, limit_command, SafetyWatchdog
from edgeai_robot_interfaces.msg import CompetitionState, EmergencyStop
from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

TASK2A = CompetitionState.PHASE_TASK2A


def passing(**overrides):
    values = {'now': 10.0, 'phase': TASK2A, 'last_state_time': 9.8,
              'auto_command': (0.1, 0.2), 'auto_time': 9.9}
    values.update(overrides)
    return GateInputs(**values)


# ---------------------------------------------------------------- pure logic

def test_fresh_input_passes_in_motion_phase():
    decision = decide(passing())
    assert (decision.linear, decision.angular) == (0.1, 0.2)
    assert decision.reasons == [] and decision.source == 'auto'


@pytest.mark.parametrize('overrides, reason', [
    ({'last_state_time': None}, 'waiting_for_competition_state'),
    ({'phase': CompetitionState.PHASE_FROZEN}, 'phase_frozen'),
    ({'phase': CompetitionState.PHASE_HANDOVER}, 'phase_handover'),
    ({'phase': CompetitionState.PHASE_ESTOP}, 'phase_estop'),
    ({'estop_active': True}, 'estop'),
    ({'last_state_time': 8.5}, 'edge_connection_lost'),
    ({'auto_time': 9.4}, 'input_timeout'),
    ({'auto_command': None, 'auto_time': None}, 'input_timeout'),
])
def test_each_closed_gate_outputs_zero(overrides, reason):
    decision = decide(passing(**overrides))
    assert (decision.linear, decision.angular) == (0.0, 0.0)
    assert reason in decision.reasons


def test_estop_applies_even_without_competition_state_requirement():
    decision = decide(passing(require_competition_state=False, last_state_time=None,
                              estop_active=True))
    assert (decision.linear, decision.angular) == (0.0, 0.0)


def test_development_without_competition_state_passes():
    decision = decide(passing(require_competition_state=False, last_state_time=None,
                              phase=None))
    assert decision.linear == 0.1


def test_manual_input_only_when_allowed_and_preferred():
    manual = {'manual_command': (0.05, 0.0), 'manual_time': 9.95}
    assert decide(passing(manual_allowed=True, **manual)).source == 'manual'
    assert decide(passing(manual_allowed=False, **manual)).source == 'auto'


def test_speed_limit_preserves_curvature():
    linear, angular = limit_command(0.3, 0.3, max_linear=0.15, max_angular=0.30)
    assert linear == pytest.approx(0.15)
    assert angular == pytest.approx(0.15)
    assert angular / linear == pytest.approx(1.0)
    assert limit_command(0.1, -0.6, 0.15, 0.30) == pytest.approx((0.05, -0.30))
    assert limit_command(0.1, 0.1, 0.15, 0.30) == (0.1, 0.1)


# ------------------------------------------------------------- ROS behaviour

def _config_dir():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'config' / 'interface_runtime.yaml').is_file():
            return str(parent / 'config')
    pytest.skip('repository config directory not found')


class Harness:

    def __init__(self, **params):
        config_dir = _config_dir()
        values = {'robot_id': 'robot_1', 'config_dir': config_dir}
        values.update(params)
        self.watchdog = SafetyWatchdog(
            namespace='robot_1', parameter_overrides=[Parameter(k, value=v)
                                                      for k, v in values.items()])
        self.node = Node('watchdog_harness')
        qos = qos_from_contract(load_runtime_contract(config_dir), 'control_state')
        self.state_pub = self.node.create_publisher(CompetitionState, '/competition/state', qos)
        self.estop_pub = self.node.create_publisher(EmergencyStop, '/competition/estop', qos)
        self.auto_pub = self.node.create_publisher(Twist, '/robot_1/cmd_vel_safe_in', 10)
        self.manual_pub = self.node.create_publisher(Twist, '/robot_1/cmd_vel_manual', 10)
        self.outputs = []
        self.node.create_subscription(Twist, '/robot_1/cmd_vel',
                                      lambda m: self.outputs.append((time.monotonic(), m)), 10)
        self.phase = TASK2A
        self.heartbeat = True
        self.command = (0.1, 0.0)
        self.feed = True
        self.node.create_timer(0.2, self._heartbeat)
        self.node.create_timer(0.05, self._feed)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.watchdog)
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def _heartbeat(self):
        if self.heartbeat:
            self.state_pub.publish(CompetitionState(run_id='run-test', phase=self.phase))

    def _feed(self):
        if self.feed:
            msg = Twist()
            msg.linear.x, msg.angular.z = self.command
            self.auto_pub.publish(msg)

    def latest(self):
        return self.outputs[-1][1] if self.outputs else None

    def wait_output(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.latest()
            if msg is not None and predicate(msg):
                return True
            time.sleep(0.02)
        return False

    def close(self):
        self.executor.shutdown()
        self.watchdog.destroy_node()
        self.node.destroy_node()


def moving(msg):
    return msg.linear.x > 0.05


def stopped(msg):
    return msg.linear.x == 0.0 and msg.angular.z == 0.0


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


def test_only_publisher_and_passes_commands(harness_factory):
    h = harness_factory()
    assert h.wait_output(moving, 3.0)
    assert len(h.node.get_publishers_info_by_topic('/robot_1/cmd_vel')) == 1


def test_zero_within_one_second_when_input_stops(harness_factory):
    h = harness_factory()
    assert h.wait_output(moving, 3.0)
    h.feed = False
    assert h.wait_output(stopped, 1.0)


def test_zero_on_estop_and_stale_generation_ignored(harness_factory):
    h = harness_factory()
    assert h.wait_output(moving, 3.0)
    h.estop_pub.publish(EmergencyStop(run_id='run-test', generation=5, active=True))
    assert h.wait_output(stopped, 1.0)
    h.estop_pub.publish(EmergencyStop(run_id='run-test', generation=4, active=False))
    time.sleep(0.5)
    assert stopped(h.latest())
    h.estop_pub.publish(EmergencyStop(run_id='run-test', generation=6, active=False))
    assert h.wait_output(moving, 1.0)


def test_zero_when_phase_frozen(harness_factory):
    h = harness_factory()
    assert h.wait_output(moving, 3.0)
    h.phase = CompetitionState.PHASE_FROZEN
    assert h.wait_output(stopped, 1.0)


def test_zero_when_edge_heartbeat_stops(harness_factory):
    h = harness_factory()
    assert h.wait_output(moving, 3.0)
    h.heartbeat = False
    assert h.wait_output(stopped, 1.5)


def test_overspeed_is_scaled(harness_factory):
    h = harness_factory()
    h.command = (0.3, 0.6)
    assert h.wait_output(lambda m: m.linear.x > 0.0, 3.0)
    time.sleep(0.3)
    msg = h.latest()
    assert msg.linear.x == pytest.approx(0.15)
    assert msg.angular.z == pytest.approx(0.30)


def test_manual_input_ignored_outside_development(harness_factory):
    h = harness_factory(run_mode='rehearsal')
    h.feed = False
    manual = Twist()
    manual.linear.x = 0.1
    end = time.monotonic() + 1.0
    while time.monotonic() < end:
        h.manual_pub.publish(manual)
        time.sleep(0.05)
    assert stopped(h.latest())


def test_disabling_competition_state_requires_development():
    rclpy.init()
    try:
        with pytest.raises(ValueError):
            SafetyWatchdog(parameter_overrides=[
                Parameter('run_mode', value='competition'),
                Parameter('require_competition_state', value=False),
                Parameter('config_dir', value=_config_dir())])
    finally:
        rclpy.shutdown()
