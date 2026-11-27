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
import math
from pathlib import Path
import re
import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_mission_interpreter.ros_node import MissionInterpreter
from edgeai_rescue_coordinator.validator_node import MissionValidator
from edgeai_robot_interfaces.action import ExecuteRescueTask
from edgeai_robot_interfaces.msg import (
    CompetitionState, EmergencyStop, MissionExecutionState, MissionIntakeResult,
    MissionPlan, MissionStep,
    MissionStepState, RescueEvent, RescueTask, RobotState, TaskReason,
)
from edgeai_robot_interfaces.srv import SubmitMission
from edgeai_system_tests.mocks.mock_robot_agent import MockRobotAgent
from edgeai_task_coordinator import mission_execution
import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String
from test_ros import Harness, wait_for


REPO = Path(__file__).resolve().parents[3]
TARGETS = (REPO / 'src/edgeai_rescue_coordinator/test/fixtures'
           / 'task1_targets.complete.example.json')


class ScriptedRobot(MockRobotAgent):
    """Return specified action failures before using the usual Mock execution."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.rescue_failures = {}
        self.rescue_failure_state = ExecuteRescueTask.Result.TERMINAL_FAILED
        self.rescue_starts = []

    def _execute_rescue(self, goal_handle):
        self.rescue_starts.append(time.monotonic())
        failures = self.rescue_failures.get(goal_handle.request.target_id)
        if failures:
            result = ExecuteRescueTask.Result(terminal_state=self.rescue_failure_state)
            return self._finish(goal_handle, result, 'failed',
                                TaskReason.NAVIGATION_FAILED, failures.pop(0))
        return super()._execute_rescue(goal_handle)


class MissionHarness(Harness):
    def __init__(self, tmp_path, targets_path, timeout, *, robot_count=2, inject=True,
                 accept_freeze=False, backoff=None):
        targets = json.loads(targets_path.read_text())
        context = {'run_id': targets['run_id'], 'map_id': targets['map']['map_id'],
                   'map_revision': targets['map']['revision'],
                   'target_revision': targets['revision']}
        self.accept_freeze = accept_freeze
        super().__init__(tmp_path, phase=CompetitionState.PHASE_TASK2B, context=context,
                         robot_count=robot_count, agent_type=ScriptedRobot, extra_overrides=[
                             Parameter('frozen_targets', value=str(targets_path)),
                             Parameter('rescue_step_timeout_s', value=float(timeout)),
                             Parameter('test_inject_rescue_outcomes', value=inject),
                             Parameter('robot_ids', value=','.join(
                                 f'robot_{i + 1}' for i in range(robot_count))),
                             *([] if backoff is None else [
                                 Parameter('rescue_retry_backoff_s', value=float(backoff))]),
                         ])
        contract = load_runtime_contract()
        snapshot = qos_from_contract(contract, 'snapshot')
        self.plans = self.control.create_publisher(MissionPlan, '/edge/mission_plan', snapshot)
        self.events = self.control.create_publisher(
            RescueEvent, '/edge/rescue_events', qos_from_contract(contract, 'event_stream'))
        self.states = []
        self.intakes = []
        self.tasks = {}
        self.assignments = []
        self.verifier_events = []
        self.rescue_status = []
        self.upstream = []
        self.control.create_subscription(
            MissionExecutionState, '/edge/mission_state', self.states.append, snapshot)
        assignment_qos = qos_from_contract(contract, 'snapshot')
        assignment_qos.depth = len(self.mocks)
        self.control.create_subscription(
            RescueTask, '/edge/rescue_tasks', self.record_task, assignment_qos)
        self.control.create_subscription(
            RescueEvent, '/edge/rescue_events', self.verifier_events.append,
            qos_from_contract(contract, 'event_stream'))
        self.control.create_subscription(
            MissionIntakeResult, '/edge/mission_intake', self.intakes.append,
            qos_from_contract(contract, 'event_stream'))
        self.control.create_subscription(
            String, '/edge/rescue/status',
            lambda msg: self.rescue_status.append(json.loads(msg.data)), snapshot)

    def _set_phase(self, request, response):
        if not self.accept_freeze:
            return super()._set_phase(request, response)
        self.finish_requests.append(request)
        response.accepted = (request.requested_phase == CompetitionState.PHASE_FROZEN
                             and not self.coordinator.mission.has_goals)
        if response.accepted:
            self.phase = request.requested_phase
            self.phase_sequence += 1
            self._publish_phase()
        response.current_phase = self.phase
        response.phase_sequence = self.phase_sequence
        return response

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
                and r.localized and r.navigation_available and not r.faults}) == len(self.mocks)
        assert wait_for(available)

    def record_task(self, task):
        self.assignments.append(task)
        self.tasks[task.target_id] = task

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
            and math.hypot(r.pose.pose.position.x - goal.pose.position.x,
                           r.pose.pose.position.y - goal.pose.position.y) <= task.zone_tolerance_m
            for rid, goal in zip(task.assigned_robot_ids, task.goal_poses))

    def wait_state(self, state):
        assert wait_for(lambda: self.states and self.states[-1].state == state, timeout=8)
        return self.states[-1]

    def wait_released(self):
        assert wait_for(lambda: not self.coordinator.mission.has_goals
                        and all(m.active_goal is None for m in self.mocks), timeout=5)


@pytest.fixture
def mission_harness(tmp_path, request):
    # The upstream validator fixture gives all normal targets the same coordinates.
    # Separate their poses here so a concurrent physical assignment is possible.
    targets = json.loads(TARGETS.read_text())
    for target in targets['targets']:
        dx, dy = {'T2': (-0.4, -0.8), 'T3': (0.9, 0.1)}.get(target['target_id'], (0, 0))
        positions = [target['marker_pose']['position'],
                     *(z['pose']['position'] for z in target['rescue_zones'])]
        for p in positions:
            p['x'] += dx
            p['y'] += dy
    options = getattr(request, 'param', {})
    if isinstance(options, (int, float)):
        options = {'timeout': options}
    options = dict(options)
    keep = options.pop('targets', None)
    if keep:
        targets['targets'] = [t for t in targets['targets'] if t['target_id'] in keep]
    targets_path = tmp_path / 'targets.json'
    targets_path.write_text(json.dumps(targets))
    rclpy.init(args=['--ros-args', '-p', f'frozen_targets:={targets_path}'])
    h = MissionHarness(tmp_path, targets_path, **{'timeout': 12, **options})
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


def test_cooperative_departure_waits_for_first_robot_to_park(mission_harness):
    h = mission_harness
    for robot in h.mocks:
        robot.linear_speed = 1.0
    h.submit([('TC1', [])])
    assert wait_for(lambda: sum(bool(m.active_goal) for m in h.mocks) == 1)
    leader, follower = sorted(h.mocks, key=lambda m: m.active_goal is None)
    assert wait_for(lambda: (
        (msg := h.coordinator.raw_states.get(leader.robot_id)) is not None
        and msg.mode == RobotState.MODE_NAVIGATING))
    assert follower.active_goal is None
    assert wait_for(lambda: h.holding('TC1'), timeout=10)
    h.outcome('TC1')
    h.wait_state(MissionExecutionState.STATE_COMPLETED)
    h.wait_released()


@pytest.mark.parametrize(('detail', 'terminal_state'), [
    ('blocked: Nav2 error 104', ExecuteRescueTask.Result.TERMINAL_FAILED),
    ('Nav2 status 6 error 208', ExecuteRescueTask.Result.TERMINAL_FAILED),
    ('blocked: collision monitor stopped the robot', ExecuteRescueTask.Result.TERMINAL_BLOCKED),
])
def test_peer_obstruction_delays_retry_without_losing_the_assignment(
        mission_harness, detail, terminal_state):
    h = mission_harness
    for robot in h.mocks:
        robot.rescue_failures['T2'] = [detail]
        robot.rescue_failure_state = terminal_state
    h.submit([('T2', [])])
    assert wait_for(lambda: any(e.event_type == RescueEvent.EVENT_BLOCKED
                                for e in h.verifier_events))
    task = h.tasks['T2']
    assert h.states[-1].state == MissionExecutionState.STATE_EXECUTING
    assert wait_for(lambda: h.holding('T2'), timeout=8)
    attempts = [stamp for robot in h.mocks for stamp in robot.rescue_starts]
    assert len(attempts) == 2 and max(attempts) - min(attempts) >= 5.0
    assert [a.task_id for a in h.assignments] == [task.task_id]
    h.outcome('T2')
    h.wait_state(MissionExecutionState.STATE_COMPLETED)
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [{'robot_count': 1}], indirect=True)
@pytest.mark.parametrize(('detail', 'terminal_state'), [
    ('Nav2 status 6 error 208', ExecuteRescueTask.Result.TERMINAL_FAILED),
    ('blocked: collision monitor stopped the robot', ExecuteRescueTask.Result.TERMINAL_BLOCKED),
])
def test_navigation_failure_without_peer_is_terminal(mission_harness, detail, terminal_state):
    h = mission_harness
    h.mocks[0].rescue_failures['T2'] = [detail]
    h.mocks[0].rescue_failure_state = terminal_state
    h.submit([('T2', []), ('T1', ['T2'])])
    steps = h.wait_state(MissionExecutionState.STATE_FAILED).steps
    assert [(s.state, s.reason.code) for s in steps] == [
        (MissionStepState.STATE_FAILED, TaskReason.NAVIGATION_FAILED),
        (MissionStepState.STATE_BLOCKED, TaskReason.DEPENDENCY_FAILED),
    ]
    assert not any(e.event_type == RescueEvent.EVENT_BLOCKED for e in h.verifier_events)
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [
    {'inject': False, 'accept_freeze': True, 'backoff': 0.2},
], indirect=True)
def test_task2a_requests_freeze_after_partial_failure_and_releases(mission_harness):
    h = mission_harness
    attempts = h.coordinator.config.settings.max_attempts + mission_execution.NAVIGATION_RETRIES
    for robot in h.mocks:
        robot.rescue_failures['T2'] = ['fixture hard failure'] * attempts
    h.phase = CompetitionState.PHASE_TASK2A
    h.phase_sequence += 1
    h._publish_phase()
    assert wait_for(lambda: h.phase == CompetitionState.PHASE_FROZEN, timeout=20)
    failed = [e for e in h.verifier_events
              if e.event_type == RescueEvent.EVENT_FAILED and e.target_id == 'T2']
    assert len(failed) == attempts
    assert any(s['status'] == 'TASK2A_FAILED' and s['failed_targets'] == ['T2']
               and set(s['completed_targets']) == {'T1', 'T3', 'TC1'} for s in h.rescue_status)
    assert 'failed targets: T2' in h.finish_requests[-1].reason
    h.wait_released()


def test_task2a_cancellation_does_not_consume_failure_budget(mission_harness):
    h = mission_harness
    h.phase = CompetitionState.PHASE_TASK2A
    h.phase_sequence += 1
    h._publish_phase()
    assert wait_for(lambda: any(m.active_goal for m in h.mocks))
    h.phase = CompetitionState.PHASE_FROZEN
    h.phase_sequence += 1
    h._publish_phase()
    h.wait_released()
    assert wait_for(lambda: any(e.event_type == RescueEvent.EVENT_CANCELLED
                                for e in h.verifier_events))
    snapshot = h.coordinator.mission.journal.read_snapshot()
    assert not snapshot['basic_failures'] and not snapshot['basic_failed']


@pytest.mark.parametrize('mission_harness', [{'inject': False, 'robot_count': 4}],
                         indirect=True)
def test_frozen_targets_use_real_dwell_and_rescore_in_task2b(mission_harness):
    h = mission_harness
    h.phase = CompetitionState.PHASE_TASK2A
    h.phase_sequence += 1
    h._publish_phase()
    ids = {'T1', 'T2', 'T3', 'TC1'}

    def completions(stage):
        return {e.target_id: e for e in h.verifier_events
                if e.event_type == RescueEvent.EVENT_COMPLETED and e.scoring_stage == stage}

    assert wait_for(lambda: len(h.assignments) >= 2, timeout=10)
    first, second = h.assignments[:2]
    assert set(first.assigned_robot_ids).isdisjoint(second.assigned_robot_ids)
    stage_a = RescueEvent.STAGE_TASK2A
    assert wait_for(lambda: set(completions(stage_a)) == ids, timeout=20)
    assert all(e.dwell_elapsed.sec >= 3 and not e.mission_id
               for e in completions(stage_a).values())
    h.wait_released()
    assert not h.states

    h.phase = CompetitionState.PHASE_TASK2B
    h.phase_sequence += 1
    h._publish_phase()
    h.start_upstream()
    rulebook = (REPO / 'docs/reference/Edge_AI_Robotics_Challenge_Rulebook_V0.md').read_text()
    instruction = re.search(r'Example instruction: “(.*?)”', rulebook).group(1)
    intake = h.submit_instruction(instruction)
    assert intake.outcome == MissionIntakeResult.OUTCOME_ACCEPTED
    terminal = h.wait_state(MissionExecutionState.STATE_COMPLETED)
    assert terminal.mission_id == intake.mission_id
    h.wait_released()
    stage_b = RescueEvent.STAGE_TASK2B
    assert set(completions(stage_b)) == ids
    for tid, event in completions(stage_b).items():
        before = completions(stage_a)[tid]
        assert event.dwell_elapsed.sec >= 3
        assert event.event_id != before.event_id and event.task_id != before.task_id
        assert event.mission_id == intake.mission_id


def stamp_s(state):
    return state.header.stamp.sec + state.header.stamp.nanosec * 1e-9


def start_task2a(h):
    h.phase = CompetitionState.PHASE_TASK2A
    h.phase_sequence += 1
    h._publish_phase()


def task2a_events(h, event_type):
    return [e for e in h.verifier_events
            if e.scoring_stage == RescueEvent.STAGE_TASK2A and e.event_type == event_type]


@pytest.mark.parametrize('mission_harness', [{'inject': False}], indirect=True)
def test_cooperative_robots_enter_one_after_another(mission_harness):
    h = mission_harness
    start_task2a(h)
    seen = {}

    def watch():
        task = h.tasks.get('TC1')
        if task is not None:
            for rid in task.assigned_robot_ids:
                state = h.coordinator.raw_states.get(rid)
                if state is not None and state.active_task_id == task.task_id:
                    seen.setdefault((rid, 'active'), stamp_s(state))
                    if state.mode == RobotState.MODE_HOLDING:
                        seen.setdefault((rid, 'holding'), stamp_s(state))
        return any(e.target_id == 'TC1' for e in task2a_events(h, RescueEvent.EVENT_COMPLETED))

    assert wait_for(watch, timeout=30)
    first, second = sorted(h.tasks['TC1'].assigned_robot_ids,
                           key=lambda rid: seen[(rid, 'active')])
    # The partner leaves only after the first robot has held its slot for holding_stable_s.
    stable = h.coordinator.config.settings.holding_stable_s
    assert seen[(second, 'active')] >= seen[(first, 'holding')] + stable - 0.05
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [{'inject': False, 'backoff': 0.2}], indirect=True)
def test_blocked_target_is_retried_then_task2a_freezes_without_it(
        mission_harness, monkeypatch):
    # The fixture models a blocked region, independent of the other robot.
    monkeypatch.setattr(mission_execution.MissionExecution, '_near_route', lambda *args: False)
    h = mission_harness
    x, y = 1.3, 2.6   # T1's rescue zone in the fixture
    for mock in h.mocks:
        mock.blocked_region = [(x - 0.3, y - 0.3), (x + 0.3, y - 0.3),
                               (x + 0.3, y + 0.3), (x - 0.3, y + 0.3)]
    start_task2a(h)
    assert wait_for(lambda: h.finish_requests, timeout=60)
    request = h.finish_requests[0]
    assert request.requested_phase == CompetitionState.PHASE_FROZEN
    assert 'T1' in request.reason
    attempts = [e for e in task2a_events(h, RescueEvent.EVENT_ASSIGNED) if e.target_id == 'T1']
    assert len(attempts) == (h.coordinator.config.settings.max_attempts
                             + mission_execution.NAVIGATION_RETRIES)
    completed = {e.target_id for e in task2a_events(h, RescueEvent.EVENT_COMPLETED)}
    assert completed == {'T2', 'T3', 'TC1'}
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [{'inject': False, 'backoff': 0.2, 'targets': ['T1']}],
                         indirect=True)
def test_retry_prefers_a_robot_that_did_not_just_fail(
        mission_harness, monkeypatch):
    # The fixture models a blocked region, independent of the other robot.
    monkeypatch.setattr(mission_execution.MissionExecution, '_near_route', lambda *args: False)
    h = mission_harness
    x, y = 1.3, 2.6   # T1's rescue zone; robot_2 starts closest to it but gets stuck
    h.mocks[1].blocked_region = [(x - 0.3, y - 0.3), (x + 0.3, y - 0.3),
                                 (x + 0.3, y + 0.3), (x - 0.3, y + 0.3)]
    h.mocks[1].linear_speed = 0.1   # reports the block on its way, not from inside the slot
    start_task2a(h)
    assert wait_for(lambda: any(
        e.target_id == 'T1' for e in task2a_events(h, RescueEvent.EVENT_COMPLETED)), timeout=40)
    done = next(e for e in task2a_events(h, RescueEvent.EVENT_COMPLETED) if e.target_id == 'T1')
    assert list(done.robot_ids) == ['robot_1']
    failed = [e for e in task2a_events(h, RescueEvent.EVENT_FAILED) if e.target_id == 'T1']
    assert [list(e.robot_ids) for e in failed] == [['robot_2']]
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [{'inject': False, 'targets': ['TC1']}],
                         indirect=True)
def test_robot_parked_on_a_zone_keeps_it(mission_harness):
    h = mission_harness
    # robot_1 still holds TC1's zone 0 from an earlier rescue. robot_2 is nearer zone 0 than
    # zone 1, but moving robot_1 sideways to zone 1 is the harder manoeuvre.
    h.mocks[0].pose[:] = [3.5, 0.1, 0.0]
    h.mocks[1].pose[:] = [2.5, 0.1, 0.0]
    start_task2a(h)
    first_active = {}

    def watch():
        task = h.tasks.get('TC1')
        if task is not None:
            for rid in task.assigned_robot_ids:
                state = h.coordinator.raw_states.get(rid)
                if state is not None and state.active_task_id == task.task_id:
                    first_active.setdefault(rid, stamp_s(state))
        return any(e.target_id == 'TC1' for e in task2a_events(h, RescueEvent.EVENT_COMPLETED))

    assert wait_for(watch, timeout=20)
    assert list(h.tasks['TC1'].assigned_robot_ids) == ['robot_1', 'robot_2']
    assert first_active['robot_1'] < first_active['robot_2']
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [{'inject': False, 'targets': ['TC1']}],
                         indirect=True)
def test_follower_without_a_clear_stop_releases_the_assignment(mission_harness, monkeypatch):
    h = mission_harness
    monkeypatch.setattr(mission_execution, 'BLOCKED_WINDOW_S', 1.0)
    execution = h.coordinator.mission
    original = execution._departure_pose

    def no_room_for_the_follower(assignment, index, robot, peers):
        return None if assignment.goals else original(assignment, index, robot, peers)

    monkeypatch.setattr(execution, '_departure_pose', no_room_for_the_follower)
    start_task2a(h)
    assert wait_for(lambda: any(
        e.target_id == 'TC1' and 'no clear stopping point' in e.reason.detail
        for e in task2a_events(h, RescueEvent.EVENT_FAILED)), timeout=15)
    h.wait_released()


@pytest.mark.parametrize('mission_harness', [{'inject': False}], indirect=True)
def test_restart_cancels_exact_goal_and_aborts_cached_plan(mission_harness):
    h = mission_harness
    h.submit([('T2', []), ('T1', ['T2'])])
    assert wait_for(lambda: h.holding('T2'), timeout=8)
    original_task = h.tasks['T2'].task_id
    h.restart_coordinator()
    state = h.wait_state(MissionExecutionState.STATE_ABORTED)
    assert state.reason.code == TaskReason.SAFETY_STOP
    assert [s.state for s in state.steps] == [MissionStepState.STATE_CANCELLED] * 2
    h.wait_released()
    assert [task.task_id for task in h.assignments] == [original_task]


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
