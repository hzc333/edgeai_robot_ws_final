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

"""Exercise public P2 ROS services with real P1 Mock acknowledgements."""

import copy
from dataclasses import asdict
import json
from pathlib import Path
import threading
import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_exploration.ros_grid import from_message
from edgeai_robot_interfaces.msg import MissionIntakeResult, MissionPlan, MissionStep, TargetArray
from edgeai_robot_interfaces.srv import FinalizeTask1, RequestReset, SetCompetitionPhase
from edgeai_system_manager.event_logger import EventLogger
from edgeai_system_manager.manager import SystemManager
from edgeai_system_tests.mocks.mock_robot_agent import MockRobotAgent
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

ROOT = Path(__file__).resolve().parents[3]
RUN = 'run-20261031T120000Z-12345678'


def wait(predicate, seconds=5):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('timed out')


@pytest.fixture
def rig(tmp_path, request):
    rclpy.init()
    params = [Parameter('run_id', value=RUN), Parameter('runs_root', value=str(tmp_path)),
              Parameter('config_dir', value=str(ROOT / 'config')),
              Parameter('run_mode', value=getattr(request, 'param', 'development'))]
    manager = SystemManager(parameter_overrides=params)
    logger = EventLogger(parameter_overrides=[
        Parameter('run_directory', value=str(tmp_path / RUN)),
        Parameter('config_dir', value=str(ROOT / 'config'))])
    robot = MockRobotAgent(namespace='/robot_1', parameter_overrides=[
        Parameter('config_dir', value=str(ROOT / 'config'))])
    harness = Node('p2_ros_test_harness')
    qos = qos_from_contract(load_runtime_contract(ROOT / 'config'), 'snapshot')
    metadata = {'run_id': RUN, 'map_id': 'map-12345678', 'revision': 1,
                'frozen': False, 'has_map': True, 'slam_running': True}
    meta_pub = harness.create_publisher(String, '/edge/map_metadata', qos)
    map_pub = harness.create_publisher(OccupancyGrid, '/map', qos)
    targets_pub = harness.create_publisher(TargetArray, '/edge/targets', qos)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width, grid.info.height, grid.info.resolution = 3, 3, 0.1
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 9

    def publish():
        meta_pub.publish(String(data=json.dumps(metadata)))
        map_pub.publish(grid)

    def freeze(request, response):
        metadata.update(frozen=True, revision=2, slam_running=False)
        manager.store.write('system/map_snapshot.json', {
            **metadata, 'grid': asdict(from_message(grid))})
        publish()
        response.success = True
        response.message = json.dumps(metadata)
        return response

    harness.create_timer(0.1, publish)
    harness.create_service(Trigger, '/edge/map_manager/freeze', freeze)
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (manager, logger, robot, harness):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    wait(lambda: manager.robot_states and manager.metadata and manager.grid
         and manager.logger_status is not None)
    try:
        yield manager, logger, robot, harness, targets_pub
    finally:
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for node in (harness, robot, logger, manager):
            node.destroy_node()
        rclpy.shutdown()


def call(harness, service, name, request):
    client = harness.create_client(service, name)
    assert client.wait_for_service(timeout_sec=3)
    future = client.call_async(request)
    wait(future.done, 10)
    result = future.result()
    harness.destroy_client(client)
    return result


def phase(harness, destination, requester=1, run=RUN):
    request = SetCompetitionPhase.Request()
    request.run_id, request.requested_phase, request.requester_type = run, destination, requester
    request.requester_node = '/p2_ros_test_harness'
    request.reason = 'integration test'
    return call(harness, SetCompetitionPhase, '/competition/set_phase', request)


def test_public_phase_permissions_safe_ack_and_event_persistence(rig):
    manager, logger, _, harness, _ = rig
    assert not phase(harness, 1, requester=2).accepted
    assert not phase(harness, 1, run='wrong').accepted
    assert phase(harness, 1).accepted
    sequence = manager.policy.state.sequence
    assert phase(harness, 1).phase_sequence == sequence
    assert phase(harness, 2, requester=2).accepted
    assert manager.safe()
    assert manager.metadata[0]['frozen']
    wait(lambda: any(e['category'] == 'phase' and e['payload']['to_phase'] == 'handover'
                     for e in logger.journal.ids.values()))
    assert manager.store.read('system/state.json')['policy']['phase'] == 2


def test_finalize_requires_p3_frozen_versions_and_is_idempotent(rig):
    manager, _, _, harness, publisher = rig
    assert phase(harness, 1).accepted
    assert phase(harness, 2).accepted
    request = FinalizeTask1.Request()
    request.run_id, request.map_id = RUN, 'map-12345678'
    request.expected_map_revision = 2
    response = call(harness, FinalizeTask1, '/edge/finalize_task1', request)
    assert not response.success
    targets = TargetArray()
    targets.header.frame_id, targets.run_id = 'map', RUN
    targets.map_id, targets.map_revision, targets.task1_frozen = 'map-12345678', 2, True
    publisher.publish(targets)
    wait(lambda: manager.targets is not None)
    response = call(harness, FinalizeTask1, '/edge/finalize_task1', request)
    assert response.success, response.message
    again = call(harness, FinalizeTask1, '/edge/finalize_task1', request)
    assert again.success
    request.expected_map_revision = 1
    assert not call(harness, FinalizeTask1, '/edge/finalize_task1', request).success
    assert phase(harness, 3).accepted
    assert manager.handover_ready
    frozen_manifest = manager.store.read('manifest.json')
    reset = RequestReset.Request()
    reset.run_id, reset.action, reset.approved_by = RUN, 1, 'test-operator'
    assert call(harness, RequestReset, '/competition/request_reset', reset).accepted
    assert manager.policy.state.phase == 7
    reset.action = 2
    response = call(harness, RequestReset, '/competition/request_reset', reset)
    assert response.accepted, response.message
    assert manager.policy.state.phase == 4
    assert not manager.policy.state.resetting_robot_ids
    assert manager.store.read('manifest.json') == frozen_manifest
    assert phase(harness, 3).accepted


def test_estop_preempts_busy_operation_and_never_auto_resumes(rig):
    manager, _, _, harness, _ = rig
    assert phase(harness, 1).accepted
    manager.operation.acquire()
    try:
        response = phase(harness, 7, requester=2)
        assert response.accepted
        assert manager.policy.state.phase == 7
    finally:
        manager.operation.release()
    assert not phase(harness, 1).accepted
    assert phase(harness, 4).accepted
    assert manager.policy.state.phase == 4
    assert phase(harness, 1).accepted


def test_reset_begin_stops_and_abort_leaves_estop(rig):
    manager, logger, _, harness, _ = rig
    assert phase(harness, 1).accepted
    request = RequestReset.Request()
    request.run_id, request.action, request.approved_by = RUN, 1, 'test-operator'
    response = call(harness, RequestReset, '/competition/request_reset', request)
    assert response.accepted, response.message
    assert manager.policy.state.phase == 7
    assert manager.policy.state.reset_count == 1
    assert call(harness, RequestReset, '/competition/request_reset', request).reset_count == 1
    request.action = 3
    assert call(harness, RequestReset, '/competition/request_reset', request).accepted
    assert manager.policy.state.phase == 7
    wait(lambda: any(e['category'] == 'reset' for e in logger.journal.ids.values()))


@pytest.mark.parametrize('rig', ['competition'], indirect=True)
def test_competition_missing_official_parameters_blocks_motion(rig):
    manager, _, _, harness, _ = rig
    response = phase(harness, 1)
    assert not response.accepted
    assert 'OFFICIAL_PARAMETER_MISSING' in response.message
    assert manager.policy.state.phase == 0


def test_finished_seals_journal_and_registers_hashes(rig):
    manager, logger, _, harness, _ = rig
    assert phase(harness, 1).accepted
    assert phase(harness, 7).accepted
    assert phase(harness, 4).accepted
    assert phase(harness, 6).accepted
    assert logger.sealed
    manifest = manager.store.read('manifest.json')
    assert any(entry['path'] == 'events.jsonl' for entry in manifest['files'])
    size = logger.journal.path.stat().st_size
    time.sleep(0.3)
    assert logger.journal.path.stat().st_size == size
    final_state = manager.store.read('system/final_state.json')
    assert final_state['policy']['phase'] == 6
    assert phase(harness, 7).accepted
    assert manager.store.read('system/state.json')['policy']['phase'] == 7
    assert manager.store.read('system/final_state.json') == final_state
    manager.transaction.verify(manager.store.read('manifest.json'))
    assert phase(harness, 6).accepted
    assert manager.store.read('system/final_state.json') == final_state
    manager.transaction.verify(manager.store.read('manifest.json'))


def test_task_timer_is_not_reset_by_estop_resume(rig):
    manager, _, _, harness, _ = rig
    assert phase(harness, 1).accepted
    time.sleep(0.1)
    assert phase(harness, 7).accepted
    assert manager.elapsed_by_phase['1'] > 0
    accumulated = manager.elapsed_by_phase['1']
    assert phase(harness, 4).accepted
    assert phase(harness, 1).accepted
    assert manager.elapsed_by_phase['1'] == accumulated


def mission_fixture():
    plan = MissionPlan()
    plan.run_id, plan.mission_id, plan.map_id = RUN, 'mission-aabbccdd', 'map-12345678'
    plan.map_revision, plan.target_revision = 2, 1
    plan.original_instruction = '救援 T1。'
    plan.model_name, plan.model_version, plan.prompt_version = 'fixture', 'v1', 'v1'
    plan.valid = True
    step = MissionStep()
    step.step_id, step.target_id, step.action_type = 'step-t1', 'T1', 1
    plan.steps = [step]
    intake = MissionIntakeResult()
    intake.run_id, intake.mission_id = RUN, plan.mission_id
    intake.original_instruction = plan.original_instruction
    intake.model_name, intake.model_version, intake.prompt_version = 'fixture', 'v1', 'v1'
    intake.raw_model_output, intake.outcome = '{"raw":"原文"}', 3
    intake.candidate_plan = copy.deepcopy(plan)
    intake.candidate_plan.valid = False
    return plan, intake


def test_mission_out_of_order_preserves_raw_input_and_deduplicates(rig):
    _, logger, _, _, _ = rig
    plan, intake = mission_fixture()
    logger.plan(plan)
    directory = logger.mission_directory(plan.mission_id)
    assert not (directory / 'mission_plan.json').exists()
    logger.intake(intake)
    assert not logger.health_error
    document = json.loads((directory / 'mission_intake.json').read_text())
    assert document['raw_model_output'] == intake.raw_model_output
    assert (directory / 'original_instruction.txt').read_text() == plan.original_instruction
    before = (directory / 'mission_plan.json').read_bytes()
    logger.intake(intake)
    logger.plan(plan)
    assert (directory / 'mission_plan.json').read_bytes() == before
    records = [json.loads(line) for line in logger.journal.path.read_text().splitlines()]
    assert sum(r['event_id'] == 'mission:' + plan.mission_id for r in records) == 1


def test_mission_mismatched_snapshot_is_rejected(rig):
    _, logger, _, _, _ = rig
    plan, intake = mission_fixture()
    intake.candidate_plan.map_revision += 1
    logger.intake(intake)
    logger.plan(plan)
    assert 'validated plan differs from intake' in logger.health_error
    assert not (logger.mission_directory(plan.mission_id) / 'mission_plan.json').exists()


def test_dead_slam_is_not_hidden_by_fresh_metadata(rig):
    manager, _, _, _, _ = rig
    with manager.state_guard:
        metadata, stamp = manager.metadata
        broken = {**metadata, 'slam_running': False}
        manager.metadata = broken, stamp
        with pytest.raises(ValueError, match='SLAM process unavailable'):
            manager.map_metadata()
        manager.metadata = metadata, stamp
