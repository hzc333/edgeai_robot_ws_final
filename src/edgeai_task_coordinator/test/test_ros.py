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

"""Real DDS/actions with the existing robot mock; no private substitute interfaces."""

import json
import threading
import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState, EmergencyStop
from edgeai_robot_interfaces.srv import SetCompetitionPhase
from edgeai_system_tests.mocks.mock_robot_agent import MockRobotAgent
from edgeai_task_coordinator.coordinator import Coordinator
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


def wait_for(predicate, timeout=15):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


class Harness:
    def __init__(self, tmp_path):
        self.control = Node('coordinator_test_control')
        self.started = time.monotonic()
        self.sim_time = 100.0
        self.heartbeat = True
        self.phase = CompetitionState.PHASE_TASK2A
        self.phase_sequence = 1
        self.generation = 1
        self.retired = []
        self.control_qos = qos_from_contract(load_runtime_contract(), 'control_state')
        self.phase_pub = self.control.create_publisher(
            CompetitionState, '/competition/state', self.control_qos)
        self.estop_pub = self.control.create_publisher(
            EmergencyStop, '/competition/estop', self.control_qos)
        self.finish_requests = []
        self.control.create_service(SetCompetitionPhase, '/competition/set_phase', self._set_phase)
        self.clock_pub = self.control.create_publisher(Clock, '/clock', 10)
        self.control.create_timer(0.02, self._publish_clock)
        self.control.create_timer(0.2, self._publish_phase)
        self.snapshots = []
        self.control.create_subscription(String, '/edge/midterm_rescue/status',
                                         lambda m: self.snapshots.append(json.loads(m.data)),
                                         qos_from_contract(load_runtime_contract(), 'snapshot'))
        overrides = [Parameter('use_sim_time', value=True),
                     Parameter('require_map', value=False),
                     Parameter('test_mode', value=True),
                     Parameter('require_collision_monitor', value=False),
                     Parameter('output_dir', value=str(tmp_path))]
        self.overrides = overrides
        self.coordinator = Coordinator(parameter_overrides=overrides)
        self.mocks = [MockRobotAgent(namespace=f'/robot_{i + 1}', parameter_overrides=[
            Parameter('robot_id', value=f'robot_{i + 1}'),
            Parameter('use_sim_time', value=True),
            Parameter('initial_x', value=0.5 + i * 0.7),
            Parameter('initial_y', value=0.5),
            Parameter('linear_speed', value=12.0),
            Parameter('angular_speed', value=12.0),
        ]) for i in range(2)]
        # Mirror production: the coordinator has a serial executor; the robot's blocking action
        # servers run independently, as they do in their own real processes.
        self.executor = SingleThreadedExecutor()
        self.mock_executor = MultiThreadedExecutor(num_threads=6)
        for node in (self.control, self.coordinator, self.coordinator.registry,
                     self.coordinator.verifier_node):
            self.executor.add_node(node)
        for node in self.mocks:
            self.mock_executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.mock_thread = threading.Thread(target=self.mock_executor.spin, daemon=True)
        self.thread.start()
        self.mock_thread.start()

    def _publish_clock(self):
        self.sim_time = 100 + 2 * (time.monotonic() - self.started)
        msg = Clock()
        msg.clock.sec = int(self.sim_time)
        msg.clock.nanosec = int(self.sim_time % 1 * 1e9)
        self.clock_pub.publish(msg)

    def _publish_phase(self):
        if self.heartbeat:
            msg = CompetitionState(run_id='coordinator-test-run', map_id='coordinator-test-map',
                                   map_revision=1,
                                   phase=self.phase, phase_sequence=self.phase_sequence)
            msg.header.stamp.sec = int(self.sim_time)
            msg.header.stamp.nanosec = int(self.sim_time % 1 * 1e9)
            self.phase_pub.publish(msg)

    def _set_phase(self, request, response):
        self.finish_requests.append(request)
        response.accepted = bool(self.coordinator.engine
                                 and self.coordinator.engine.status == 'DONE')
        if response.accepted:
            self.phase = request.requested_phase
            self.phase_sequence += 1
            self._publish_phase()
        response.current_phase = self.phase
        response.phase_sequence = self.phase_sequence
        return response

    def active(self):
        return self.coordinator.engine and self.coordinator.engine.status == 'ACTIVE'

    def restart_coordinator(self):
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        for node in (self.control, self.coordinator, self.coordinator.registry,
                     self.coordinator.verifier_node):
            self.executor.remove_node(node)
        # Simulate process loss without graceful action cancellation. The robot keeps
        # the original action; the replacement must cancel its exact UUID.
        self.coordinator.timer.cancel()
        self.coordinator.journal.close()
        self.coordinator.map_worker.shutdown(wait=False, cancel_futures=True)
        self.retired.append(self.coordinator)
        self.coordinator = Coordinator(parameter_overrides=self.overrides)
        self.executor = SingleThreadedExecutor()
        for node in (self.control, self.coordinator, self.coordinator.registry,
                     self.coordinator.verifier_node):
            self.executor.add_node(node)
        # Drain old client futures without ticking the retired coordinator.
        # Real SIGKILL/process restart is tested separately in Gazebo.
        for retired in self.retired:
            self.executor.add_node(retired)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def close(self):
        self.phase, self.phase_sequence = CompetitionState.PHASE_FROZEN, self.phase_sequence + 1
        self.heartbeat = True
        self._publish_phase()
        wait_for(lambda: all(m.active_goal is None for m in self.mocks)
                 and not self.coordinator.handles and not self.coordinator.goal_records, timeout=3)
        self.coordinator.timer.cancel()

        def settled():
            for node in (self.coordinator, *self.retired):
                services = [node.phase_client, *node.cancel_clients.values(),
                            *(entry[0] for entry in node.result_clients.values())]
                if any(c._pending_requests for c in services):
                    return False
                for client in node.action_clients.values():
                    if (client._pending_goal_requests or client._pending_cancel_requests
                            or client._pending_result_requests):
                        return False
            return True

        clean = wait_for(settled, timeout=3)
        time.sleep(0.3)
        self.executor.shutdown(timeout_sec=5)
        self.mock_executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        self.mock_thread.join(timeout=5)
        self.coordinator.close()
        for retired in self.retired:
            retired.close()
        for mock in self.mocks:
            mock.destroy_node()
        self.control.destroy_node()
        assert clean, 'unsettled ROS requests at test teardown'


@pytest.fixture
def harness(tmp_path):
    rclpy.init()
    h = Harness(tmp_path)
    try:
        yield h
    finally:
        try:
            h.close()
        finally:
            rclpy.shutdown()


def test_two_rescues_over_real_actions_and_persisted_evidence(harness):
    h = harness
    assert wait_for(lambda: h.coordinator.engine and h.coordinator.engine.status == 'DONE', 35)
    directory = h.coordinator.journal.directory
    events = [json.loads(line) for line in (directory / 'events.jsonl').read_text().splitlines()]
    completed = [e for e in events if e['type'] == 'COMPLETED']
    assert [e['target_id'] for e in completed] == ['S1', 'C1']
    for event in completed:
        region = event['verification_region']
        assert event['dwell_elapsed_s'] >= 10
        assert region['center'] == ([1.0, 4.1] if event['target_id'] == 'S1' else [4.1, 1.0])
    assert len(set(completed[1]['robot_ids'])) == 2
    assert not h.coordinator.goal_records
    assert all(m.active_goal is None for m in h.mocks)
    assert all(m.velocity == (0.0, 0.0) for m in h.mocks)
    assert h.snapshots[-1]['collision_status'] == 'UNKNOWN'
    assert wait_for(lambda: len(h.finish_requests) == 1)
    assert h.finish_requests[0].requester_type == SetCompetitionPhase.Request.REQUESTER_SYSTEM
    assert h.finish_requests[0].requested_phase == CompetitionState.PHASE_FROZEN


def test_s1_navigation_failure_reassigns_to_other_robot_and_finishes(harness):
    h = harness
    assert wait_for(h.active)
    first = h.coordinator.engine.assignment[0]
    mock = next(m for m in h.mocks if m.robot_id == first)
    with mock.lock:
        mock.fail_navigation_after_s = 0.01
    assert wait_for(lambda: h.coordinator.engine.failures.get('S1', 0) == 1)
    assert wait_for(lambda: h.active() and h.coordinator.engine.target.target_id == 'S1'
                    and h.coordinator.engine.assignment[0] != first)
    with mock.lock:
        mock.fail_navigation_after_s = 0.0
    assert wait_for(lambda: h.coordinator.engine.status == 'DONE', 35)
    assert h.coordinator.engine.completed == {'S1', 'C1'}
    assert h.coordinator.engine.attempts['S1'] == 2


def test_idle_peer_localization_loss_cancels_and_recovers_without_spending_retry(harness):
    h = harness
    assert wait_for(h.active)
    assigned = h.coordinator.engine.assignment[0]
    peer = next(m for m in h.mocks if m.robot_id != assigned)
    with peer.lock:
        peer.localized = False
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert all(m.active_goal is None for m in h.mocks)
    assert h.coordinator.engine.failures.get('S1', 0) == 0
    with peer.lock:
        peer.localized = True
    assert wait_for(lambda: h.coordinator.engine.status == 'DONE', 35)


def test_freeze_cancels_real_mock_action_and_waits_for_authorization(harness):
    h = harness
    assert wait_for(h.active)
    assert wait_for(lambda: bool(h.coordinator.handles))
    h.phase, h.phase_sequence = CompetitionState.PHASE_FROZEN, 2
    h._publish_phase()
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: all(m.active_goal is None for m in h.mocks))
    assert not h.coordinator.engine.completed


def test_estop_old_generation_cannot_release_stop(harness):
    h = harness
    assert wait_for(h.active)
    h.estop_pub.publish(EmergencyStop(generation=2, active=True, reason='coordinator test'))
    assert wait_for(lambda: h.coordinator.estop)
    h.estop_pub.publish(EmergencyStop(generation=1, active=False))
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert h.coordinator.estop
    assert not h.coordinator.engine.completed


def test_edge_heartbeat_loss_cancels_without_scoring(harness):
    h = harness
    assert wait_for(h.active)
    assert wait_for(lambda: bool(h.coordinator.handles))
    h.heartbeat = False
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: all(m.active_goal is None for m in h.mocks))
    assert not h.coordinator.engine.completed


def test_shutdown_cancels_even_while_phase_heartbeat_continues(harness):
    h = harness
    assert wait_for(h.active)
    assert wait_for(lambda: bool(h.coordinator.handles))
    h.coordinator.shutdown_requested = True
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: all(m.active_goal is None for m in h.mocks))
    assert h.heartbeat
    assert not h.coordinator.engine.completed


def test_freeze_then_authorized_same_run_resume_completes(harness):
    h = harness
    assert wait_for(lambda: h.active() and bool(h.coordinator.handles))
    h.phase, h.phase_sequence = CompetitionState.PHASE_FROZEN, 2
    h._publish_phase()
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: not h.coordinator.goal_records)
    time.sleep(0.4)
    assert h.coordinator.engine.status == 'RECOVERING'
    h.phase, h.phase_sequence = CompetitionState.PHASE_TASK2A, 3
    h._publish_phase()
    assert wait_for(lambda: h.coordinator.engine.status == 'DONE', 35)
    assert h.coordinator.engine.attempts['S1'] == 2
    events = [json.loads(s) for s in
              (h.coordinator.journal.directory / 'events.jsonl').read_text().splitlines()]
    assert sum(e['type'] == 'COMPLETED' and e['target_id'] == 'S1' for e in events) == 1
    assert any(e['type'] == 'RECOVERED' for e in events)


def test_process_restart_preserves_s1_and_recovers_c1(harness):
    h = harness
    assert wait_for(lambda: h.active() and h.coordinator.engine.completed == {'S1'}
                    and bool(h.coordinator.goal_records), 25)
    attempts = h.coordinator.engine.attempts['C1']
    h.restart_coordinator()
    assert wait_for(lambda: h.coordinator.engine and h.coordinator.engine.status == 'DONE', 35)
    assert h.coordinator.engine.attempts['S1'] == 1
    assert h.coordinator.engine.attempts['C1'] > attempts
    events = [json.loads(s) for s in
              (h.coordinator.journal.directory / 'events.jsonl').read_text().splitlines()]
    assert [e['target_id'] for e in events if e['type'] == 'COMPLETED'] == ['S1', 'C1']
    assert not h.coordinator.goal_records and not h.coordinator.orphaned


class PhysicsFeed:
    """Inject observation loss/contacts over the same serialized ROS tick path."""

    overflow = False

    def __init__(self, harness):
        self.h = harness
        self.available = False
        self.contacts = []

    def drain(self):
        if not self.available:
            return []
        return [(self.h.sim_time, time.monotonic(), {'robot_1': 9, 'robot_2': 9},
                 ['robot_1', 'robot_2'], self.contacts)]

    def close(self):
        pass


def test_missing_physics_stops_then_recovers_and_contact_latches(harness):
    h = harness
    assert wait_for(lambda: h.active() and bool(h.coordinator.handles))
    feed = PhysicsFeed(h)
    h.coordinator.contact_feed = feed
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: not h.coordinator.goal_records)
    assert h.snapshots[-1]['collision_status'] == 'UNKNOWN'
    feed.available = True
    assert wait_for(h.active)
    feed.contacts = [('world::robot_1::body::solid', 'world::robot_2::body::solid')]
    assert wait_for(lambda: h.coordinator.collision.count > 0)
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: not h.coordinator.goal_records)
    feed.contacts = []
    assert wait_for(lambda: h.snapshots[-1]['collision_status'] == 'CLEAR')
    time.sleep(1.3)
    assert h.coordinator.engine.status == 'RECOVERING'
    assert h.snapshots[-1]['collision']['ever_contact']
    events = [json.loads(s) for s in
              (h.coordinator.journal.directory / 'events.jsonl').read_text().splitlines()]
    assert sum(e['type'] == 'COLLISION' for e in events) == 1
    assert not h.coordinator.engine.completed


def test_estop_release_requires_new_task2a_phase_authorization(harness):
    h = harness
    assert wait_for(lambda: h.active() and bool(h.coordinator.handles))
    h.estop_pub.publish(EmergencyStop(generation=2, active=True))
    assert wait_for(lambda: h.coordinator.estop)
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    h.estop_pub.publish(EmergencyStop(generation=3, active=False))
    assert wait_for(lambda: not h.coordinator.estop)
    time.sleep(1.3)
    assert h.coordinator.engine.status == 'RECOVERING'
    h.phase_sequence += 1
    h._publish_phase()
    assert wait_for(h.active)


def test_heartbeat_recovery_resumes_same_run_without_reset(harness):
    h = harness
    assert wait_for(lambda: h.active() and bool(h.coordinator.handles))
    h.heartbeat = False
    assert wait_for(lambda: h.coordinator.engine.status == 'RECOVERING')
    assert wait_for(lambda: not h.coordinator.goal_records)
    h.heartbeat = True
    assert wait_for(h.active)
    assert h.coordinator.engine.run_id == 'coordinator-test-run'
    assert h.coordinator.engine.attempts['S1'] >= 2


def test_lost_orphan_result_response_can_be_queried_again():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from unittest.mock import Mock

    from edgeai_robot_interfaces.action import ExecuteNavigationGoal

    old = Future()
    client = Mock()
    replacement = Future()
    client.call_async.return_value = replacement
    record = {'robot_id': 'robot_1', 'kind': 'navigation', 'uuid': [0] * 16,
              'result_requested_wall': time.monotonic() - 3}
    stub = SimpleNamespace(
        result_clients={('robot_1', 'navigation'): (
            client, ExecuteNavigationGoal.Impl.GetResultService)}, orphan_futures={'old': old})
    Coordinator._reconcile_orphan(stub, 'old', record)
    assert old.cancelled() and 'old' not in stub.orphan_futures
    Coordinator._reconcile_orphan(stub, 'old', record)
    assert stub.orphan_futures['old'] is replacement
    assert client.call_async.call_count == 1


def test_late_acceptance_from_retired_attempt_is_always_cancelled():
    from concurrent.futures import Future
    from types import SimpleNamespace
    from unittest.mock import Mock

    result = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
    response = Future()
    response.set_result(handle)
    stub = SimpleNamespace(
        handles={}, goal_records={}, cancel_pending=set(), _now=lambda: 1, _ended=Mock(),
        engine=SimpleNamespace(slots={}, status='ACTIVE', transit_accepted=Mock()))
    Coordinator._accepted(stub, 'old-attempt', response, 'navigation')
    assert 'old-attempt' in stub.cancel_pending
