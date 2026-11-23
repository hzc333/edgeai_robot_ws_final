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

"""Exercise the real allocator, scheduler and robot actions with a test verifier."""

import json
from pathlib import Path
import re

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_mission_interpreter.ros_node import MissionInterpreter
from edgeai_rescue_coordinator.validator_node import MissionValidator
from edgeai_robot_interfaces.msg import (
    CompetitionState, EmergencyStop, MissionExecutionState, MissionIntakeResult,
    MissionPlan, MissionStep,
    MissionStepState, RescueEvent, RescueTask, RobotState, TaskReason,
)
from edgeai_robot_interfaces.srv import SubmitMission
import pytest
import rclpy
from rclpy.parameter import Parameter
from test_ros import Harness, wait_for


REPO = Path(__file__).resolve().parents[3]
TARGETS = (REPO / 'src/edgeai_rescue_coordinator/test/fixtures'
           / 'task1_targets.complete.example.json')


class MissionHarness(Harness):
    def __init__(self, tmp_path, timeout):
        targets = json.loads(TARGETS.read_text())
        context = {'run_id': targets['run_id'], 'map_id': targets['map']['map_id'],
                   'map_revision': targets['map']['revision'],
                   'target_revision': targets['revision']}
        super().__init__(tmp_path, phase=CompetitionState.PHASE_TASK2B, context=context,
                         extra_overrides=[
                             Parameter('task2b_frozen_targets', value=str(TARGETS)),
                             Parameter('task2b_step_timeout_s', value=float(timeout)),
                         ])
        contract = load_runtime_contract()
        snapshot = qos_from_contract(contract, 'snapshot')
        self.plans = self.control.create_publisher(MissionPlan, '/edge/mission_plan', snapshot)
        self.events = self.control.create_publisher(
            RescueEvent, '/edge/rescue_events', qos_from_contract(contract, 'event_stream'))
        self.states = []
        self.intakes = []
        self.tasks = {}
        self.upstream = []
        self.control.create_subscription(
            MissionExecutionState, '/edge/mission_state', self.states.append, snapshot)
        assignment_qos = qos_from_contract(contract, 'snapshot')
        assignment_qos.depth = len(self.mocks)
        self.control.create_subscription(
            RescueTask, '/edge/rescue_tasks', lambda m: self.tasks.update({m.target_id: m}),
            assignment_qos)
        self.control.create_subscription(
            MissionIntakeResult, '/edge/mission_intake', self.intakes.append,
            qos_from_contract(contract, 'event_stream'))

    def start_upstream(self):
        self.upstream = [MissionInterpreter(), MissionValidator()]
        for node in self.upstream:
            self.executor.add_node(node)
        self.submit_client = self.control.create_client(SubmitMission, '/edge/submit_mission')
        assert self.submit_client.wait_for_service(timeout_sec=5)
        validator = self.upstream[1]

        # Count robots as the validator does: a mock's first state may predate the run state.
        def available():
            state, robots = validator.state, validator.robots
            return state is not None and robots is not None and len({
                r.robot_id for r in robots.robots if r.run_id == state.run_id
                and r.localized and r.navigation_available and not r.faults}) == 2
        assert wait_for(available)

    def submit_instruction(self, instruction):
        future = self.submit_client.call_async(SubmitMission.Request(
            run_id=self.context['run_id'], original_instruction=instruction))
        assert wait_for(future.done, timeout=5)
        response = future.result()
        assert response.accepted
        assert wait_for(lambda: any(
            msg.mission_id == response.mission_id for msg in self.intakes), timeout=8)
        return next(msg for msg in self.intakes if msg.mission_id == response.mission_id)

    def close(self):
        try:
            super().close()
        finally:
            for node in self.upstream:
                node.destroy_node()

    def submit(self, steps, mission_id='test-mission', **context):
        plan = MissionPlan(**{**self.context, **context}, mission_id=mission_id, valid=True,
                           steps=[MissionStep(step_id=s, target_id=s,
                                              action_type=MissionStep.ACTION_RESCUE,
                                              predecessor_step_ids=p) for s, p in steps])
        plan.header.frame_id = 'map'
        self.plans.publish(plan)

    def outcome(self, step, event_type=RescueEvent.EVENT_COMPLETED, **fields):
        task = self.tasks[step]
        values = {'run_id': task.run_id, 'mission_id': task.mission_id, 'step_id': task.step_id,
                  'task_id': task.task_id, 'target_id': task.target_id,
                  'scoring_stage': RescueEvent.STAGE_TASK2B, 'event_type': event_type,
                  'robot_ids': list(task.assigned_robot_ids)}
        self.events.publish(RescueEvent(**{**values, **fields}))

    def holding(self, step):
        task = self.tasks.get(step)
        return task is not None and all(
            (r := self.coordinator.raw_states.get(rid)) is not None
            and r.mode == RobotState.MODE_HOLDING and r.active_task_id == task.task_id
            for rid in task.assigned_robot_ids)

    def wait_state(self, state):
        assert wait_for(lambda: self.states and self.states[-1].state == state, timeout=8)
        return self.states[-1]

    def wait_released(self):
        assert wait_for(lambda: not self.coordinator.mission.assigned
                        and all(m.active_goal is None for m in self.mocks), timeout=5)


@pytest.fixture
def mission_harness(tmp_path, request):
    rclpy.init(args=['--ros-args', '-p', f'frozen_targets:={TARGETS}'])
    h = MissionHarness(tmp_path, getattr(request, 'param', 12))
    try:
        yield h
    finally:
        try:
            h.close()
        finally:
            rclpy.shutdown()


def test_rulebook_dependencies_dispatch_independent_steps_in_parallel(mission_harness):
    h = mission_harness
    steps = [('T2', []), ('T1', ['T2']), ('T3', ['T2']), ('TC1', ['T1', 'T3'])]
    h.submit(steps, mission_id='wrong-context', target_revision=h.context['target_revision'] + 1)
    h.start_upstream()
    rulebook = (REPO / 'docs/reference/Edge_AI_Robotics_Challenge_Rulebook_V0.md').read_text()
    instruction = re.search(r'Example instruction: “(.*?)”', rulebook).group(1)
    intake = h.submit_instruction(instruction)
    assert intake.outcome == MissionIntakeResult.OUTCOME_ACCEPTED
    assert wait_for(lambda: h.holding('T2'), timeout=8)
    assert set(h.tasks) == {'T2'}
    assert all(state.mission_id == intake.mission_id for state in h.states)
    h.outcome('T2', scoring_stage=RescueEvent.STAGE_TASK2A)
    h.outcome('T2', task_id='old-task')
    assert not wait_for(lambda: len(h.tasks) > 1, timeout=0.4)

    h.outcome('T2')
    assert wait_for(lambda: h.holding('T1') and h.holding('T3'), timeout=8)
    assert set(h.tasks['T1'].assigned_robot_ids).isdisjoint(h.tasks['T3'].assigned_robot_ids)
    h.outcome('T1')
    assert not wait_for(lambda: 'TC1' in h.tasks, timeout=0.4)
    h.outcome('T3')
    assert wait_for(lambda: h.holding('TC1'), timeout=8)
    assert len(set(h.tasks['TC1'].assigned_robot_ids)) == 2
    h.outcome('TC1', robot_ids=list(reversed(h.tasks['TC1'].assigned_robot_ids)))
    assert not wait_for(lambda: h.states[-1].state == MissionExecutionState.STATE_COMPLETED,
                        timeout=0.4)
    h.outcome('TC1')
    assert all(step.state == MissionStepState.STATE_COMPLETED
               for step in h.wait_state(MissionExecutionState.STATE_COMPLETED).steps)
    h.wait_released()
    h.submit(steps, mission_id=intake.mission_id)
    assert not wait_for(lambda: h.states[-1].state != MissionExecutionState.STATE_COMPLETED,
                        timeout=0.4)
    invalid = h.submit_instruction('Rescue T999.')
    assert invalid.outcome == MissionIntakeResult.OUTCOME_REJECTED_BY_VALIDATOR
    assert any(issue.code == 102 and issue.target_id == 'T999' for issue in invalid.issues)
    assert not wait_for(lambda: any(
        state.mission_id == invalid.mission_id for state in h.states), timeout=0.5)
    publishers = h.control.get_publishers_info_by_topic('/edge/mission_state')
    assert [(p.node_namespace, p.node_name) for p in publishers] == [('/edge', 'task_allocator')]


@pytest.mark.parametrize('mission_harness', [2.5], indirect=True)
def test_timeout_fails_step_and_blocks_its_dependency(mission_harness):
    h = mission_harness
    h.submit([('T2', []), ('T1', ['T2'])])
    assert wait_for(lambda: h.holding('T2'), timeout=2.5)
    steps = h.wait_state(MissionExecutionState.STATE_FAILED).steps
    assert [(s.state, s.reason.code) for s in steps] == [
        (MissionStepState.STATE_FAILED, TaskReason.NAVIGATION_TIMEOUT),
        (MissionStepState.STATE_BLOCKED, TaskReason.DEPENDENCY_FAILED),
    ]
    assert set(h.tasks) == {'T2'}
    h.wait_released()


def test_verifier_failure_keeps_independent_branch_active(mission_harness):
    h = mission_harness
    h.submit([('T2', []), ('T1', []), ('T3', ['T2'])])
    assert wait_for(lambda: h.holding('T2') and h.holding('T1'), timeout=8)
    h.outcome('T2', RescueEvent.EVENT_FAILED, reason=TaskReason(code=TaskReason.ZONE_EXITED))
    assert wait_for(lambda: any(step.state == MissionStepState.STATE_BLOCKED
                                for step in h.states[-1].steps))
    assert h.states[-1].state == MissionExecutionState.STATE_EXECUTING
    h.outcome('T1')
    steps = {s.target_id: s for s in h.wait_state(MissionExecutionState.STATE_FAILED).steps}
    assert steps['T2'].reason.code == TaskReason.ZONE_EXITED
    assert steps['T3'].reason.code == TaskReason.DEPENDENCY_FAILED
    assert steps['T1'].state == MissionStepState.STATE_COMPLETED
    assert 'T3' not in h.tasks
    h.wait_released()


@pytest.mark.parametrize('reason', [TaskReason.SAFETY_STOP, TaskReason.PHASE_CHANGED])
def test_interrupt_aborts_mission_and_releases_robot(mission_harness, reason):
    h = mission_harness
    h.submit([('T2', []), ('T1', ['T2'])])
    assert wait_for(lambda: h.holding('T2'), timeout=8)
    if reason == TaskReason.SAFETY_STOP:
        h.estop_pub.publish(EmergencyStop(run_id=h.context['run_id'], active=True, generation=1))
    else:
        h.phase = CompetitionState.PHASE_FROZEN
        h.phase_sequence += 1
        h._publish_phase()
    state = h.wait_state(MissionExecutionState.STATE_ABORTED)
    assert state.reason.code == reason
    assert all(s.state == MissionStepState.STATE_CANCELLED for s in state.steps)
    assert set(h.tasks) == {'T2'}
    h.wait_released()
