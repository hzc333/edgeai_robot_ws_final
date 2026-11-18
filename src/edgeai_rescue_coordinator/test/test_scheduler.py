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

"""Contract, transition and generated-DAG tests for card 03."""

from copy import deepcopy
import json
from pathlib import Path
import random
import re

from edgeai_rescue_coordinator.scheduler import DependencyScheduler
from edgeai_rescue_coordinator.scheduler import MissionState as M
from edgeai_rescue_coordinator.scheduler import Reason as R
from edgeai_rescue_coordinator.scheduler import StepState as S
import pytest

ROOT = Path(__file__).resolve().parents[3]


def step(sid, preds=(), priority=0, target='T1'):
    """Create a message-shaped step."""
    return {'step_id': sid, 'target_id': target, 'action_type': 1,
            'priority': priority, 'predecessor_step_ids': list(preds)}


def make_plan(steps):
    """Convert the repository example to a validated message-shaped fixture."""
    log = json.loads((ROOT / 'examples/interfaces/mission_plan.example.json').read_text())
    result = {k: log[k] for k in ('run_id', 'mission_id', 'map_id', 'map_revision',
                                  'target_revision', 'original_instruction')}
    result.update(header={'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': 'map'},
                  model_name=log['model']['name'], model_version=log['model']['version'],
                  prompt_version=log['model']['prompt_version'], valid=True,
                  steps=deepcopy(steps))
    return result


@pytest.fixture
def plan():
    """Use rulebook 6.8's four steps from the repository example."""
    log = json.loads((ROOT / 'examples/interfaces/mission_plan.example.json').read_text())
    return make_plan([step(s['step_id'], s['predecessor_step_ids'],
                           s['priority'], s['target_id']) for s in log['steps']])


def event(plan, sid, kind, tick=1):
    """Supply external identity, assignment and timestamp."""
    return {'run_id': plan['run_id'], 'mission_id': plan['mission_id'],
            'step_id': sid, 'task_id': 'task-' + sid, 'robot_ids': ['external-robot'],
            'kind': kind, 'stamp': {'sec': tick, 'nanosec': 0}}


def ids(scheduler):
    """List ready step IDs."""
    return [s['step_id'] for s in scheduler.ready_steps()]


def states(scheduler):
    """Index snapshot records by ID."""
    return {s['step_id']: s for s in scheduler.snapshot()['steps']}


def finish(scheduler, plan, sid, tick=1):
    """Deliver start and completion explicitly."""
    assert scheduler.apply(event(plan, sid, 'start', tick))
    assert scheduler.apply(event(plan, sid, 'complete', tick + 1))


def contract(name):
    """Read constants and fields from the shared msg file."""
    text = (ROOT / 'src/edgeai_robot_interfaces/msg' / name).read_text()
    constants = {k: int(v) for k, v in re.findall(r'^uint\d+ (\w+)=(\d+)$', text, re.M)}
    fields = {line.split()[1] for line in text.splitlines()
              if line and not line.startswith('#') and '=' not in line}
    return constants, fields


def test_all_constants_match_messages():
    """Compare all seven/five states and every TaskReason constant."""
    for enum, name, prefix in ((S, 'MissionStepState.msg', 'STATE_'),
                               (M, 'MissionExecutionState.msg', 'STATE_'),
                               (R, 'TaskReason.msg', '')):
        assert {prefix + e.name: int(e) for e in enum} == contract(name)[0]


def test_snapshot_fields_and_input_output_isolation(plan):
    """Preserve inputs and expose every nested message field with external metadata."""
    original = deepcopy(plan)
    scheduler = DependencyScheduler(plan)
    start = event(plan, 'step-t2', 'start', 8)
    assert scheduler.apply(start)
    assert scheduler.apply(event(plan, 'step-t2', 'complete', 9))
    snapshot = scheduler.snapshot()
    assert set(snapshot) == contract('MissionExecutionState.msg')[1]
    for record in snapshot['steps']:
        assert set(record) == contract('MissionStepState.msg')[1]
        assert set(record['reason']) == contract('TaskReason.msg')[1]
        assert set(record['completed_at']) == {'sec', 'nanosec'}
    done = states(scheduler)['step-t2']
    assert done['task_id'] == start['task_id']
    assert done['robot_ids'] == start['robot_ids']
    assert done['completed_at'] == {'sec': 9, 'nanosec': 0}
    assert snapshot['header']['stamp'] == done['completed_at']
    assert plan == original
    frozen = scheduler.snapshot()
    plan['steps'].clear()
    start['robot_ids'].append('changed')
    snapshot['steps'].clear()
    scheduler.ready_steps()[0]['priority'] = 99
    assert scheduler.snapshot() == frozen


def test_rulebook_6_8_release_sequence(plan):
    """Release T2, then T1/T3, and finally TC1 after both predecessors."""
    scheduler = DependencyScheduler(plan)
    assert scheduler.snapshot()['state'] == M.IDLE
    assert ids(scheduler) == ['step-t2']
    assert not scheduler.apply(event(plan, 'step-tc1', 'start'))
    finish(scheduler, plan, 'step-t2')
    assert ids(scheduler) == ['step-t1', 'step-t3']
    finish(scheduler, plan, 'step-t3', 3)
    assert ids(scheduler) == ['step-t1']
    finish(scheduler, plan, 'step-t1', 5)
    assert ids(scheduler) == ['step-tc1']
    finish(scheduler, plan, 'step-tc1', 7)
    assert scheduler.snapshot()['state'] == M.COMPLETED
    assert ids(scheduler) == []


def test_priority_ties_cannot_bypass_dependencies():
    """Sort descending priority and lexical step_id while preserving prerequisites."""
    plan = make_plan([step('z', priority=2), step('b', priority=8),
                      step('a', priority=8), step('next', ['z'], 99)])
    scheduler = DependencyScheduler(plan)
    assert ids(scheduler) == ['a', 'b', 'z']
    assert not scheduler.apply(event(plan, 'next', 'start'))
    finish(scheduler, plan, 'z')
    assert ids(scheduler) == ['next', 'a', 'b']


@pytest.mark.parametrize('kind', ['fail', 'cancel'])
def test_transitive_blocking_keeps_independent_branch_running(kind):
    """Propagate blocked state despite reverse plan order, then reach FAILED."""
    plan = make_plan([step('c', ['b']), step('b', ['a']), step('a'), step('free')])
    scheduler = DependencyScheduler(plan)
    assert scheduler.apply(event(plan, 'a', 'start'))
    assert scheduler.apply(event(plan, 'a', kind, 2))
    assert states(scheduler)['a']['state'] == (S.FAILED if kind == 'fail' else S.CANCELLED)
    for sid in ('b', 'c'):
        assert states(scheduler)[sid]['state'] == S.BLOCKED
        assert states(scheduler)[sid]['reason']['code'] == R.DEPENDENCY_FAILED
    assert ids(scheduler) == ['free']
    assert scheduler.snapshot()['state'] == M.EXECUTING
    finish(scheduler, plan, 'free', 3)
    assert scheduler.snapshot()['state'] == M.FAILED


def test_external_failure_reason_and_all_blocked_terminal(plan):
    """Pass through executor reason and terminate when every remaining step is blocked."""
    scheduler = DependencyScheduler(plan)
    scheduler.apply(event(plan, 'step-t2', 'start'))
    failure = event(plan, 'step-t2', 'fail', 2)
    failure['reason'] = {'code': int(R.NAVIGATION_TIMEOUT), 'detail': 'external timeout'}
    assert scheduler.apply(failure)
    assert scheduler.snapshot()['state'] == M.FAILED
    assert states(scheduler)['step-t2']['reason'] == failure['reason']


@pytest.mark.parametrize('kind,code', [
    ('phase_changed', R.PHASE_CHANGED), ('safety_stop', R.SAFETY_STOP),
])
def test_interrupt_aborts_and_cancels_uncompleted(plan, kind, code):
    """Preserve completion and cancel READY, IN_PROGRESS and WAITING steps."""
    scheduler = DependencyScheduler(plan)
    finish(scheduler, plan, 'step-t2')
    scheduler.apply(event(plan, 'step-t1', 'start', 3))
    completed = states(scheduler)['step-t2']
    assert scheduler.apply(event(plan, '', kind, 4))
    assert scheduler.snapshot()['state'] == M.ABORTED
    assert scheduler.snapshot()['reason']['code'] == code
    assert states(scheduler)['step-t2'] == completed
    for sid in ('step-t1', 'step-t3', 'step-tc1'):
        assert states(scheduler)[sid]['state'] == S.CANCELLED
        assert states(scheduler)[sid]['reason']['code'] == code
    assert not scheduler.apply(event(plan, 'step-t1', 'complete', 5))
    assert ids(scheduler) == []


@pytest.mark.parametrize('kind,code', [
    ('phase_changed', R.PHASE_CHANGED), ('safety_stop', R.SAFETY_STOP),
])
def test_abort_also_cancels_failed_and_blocked_in_active_mission(kind, code):
    """Preserve all terminal records and reasons; cancel only nonterminal steps."""
    plan = make_plan([step('a'), step('b', ['a']), step('cancelled'),
                      step('done'), step('active'), step('free'),
                      step('waiting', ['active'])])
    scheduler = DependencyScheduler(plan)
    assert scheduler.apply(event(plan, 'a', 'start'))
    failure = event(plan, 'a', 'fail', 2)
    failure['reason'] = {'code': int(R.NAVIGATION_FAILED), 'detail': 'original failure'}
    assert scheduler.apply(failure)
    assert scheduler.apply(event(plan, 'cancelled', 'start', 3))
    cancellation = event(plan, 'cancelled', 'cancel', 4)
    cancellation['reason'] = {'code': int(R.USER_REQUEST), 'detail': 'original cancellation'}
    assert scheduler.apply(cancellation)
    finish(scheduler, plan, 'done', 5)
    assert scheduler.apply(event(plan, 'active', 'start', 7))
    before = states(scheduler)
    assert {s['state'] for s in before.values()} == set(S)
    assert scheduler.apply(event(plan, '', kind, 8))
    after = states(scheduler)
    for sid in ('a', 'b', 'cancelled', 'done'):
        assert after[sid] == before[sid]
    for sid in ('active', 'free', 'waiting'):
        assert after[sid]['state'] == S.CANCELLED
        assert after[sid]['reason'] == {'code': int(code), 'detail': ''}
    assert scheduler.snapshot()['state'] == M.ABORTED
    assert scheduler.snapshot()['reason'] == {'code': int(code), 'detail': ''}


def test_dirty_events_never_regress_or_count_twice(plan):
    """Ignore early completion, unknown IDs, duplicates, stale time and wrong assignments."""
    scheduler = DependencyScheduler(plan)
    initial = scheduler.snapshot()
    assert not scheduler.apply(event(plan, 'step-t2', 'complete', 2))
    assert not scheduler.apply(event(plan, 'missing', 'start'))
    assert scheduler.snapshot() == initial
    assert scheduler.apply(event(plan, 'step-t2', 'start', 3))
    active = scheduler.snapshot()
    assert not scheduler.apply(event(plan, 'step-t2', 'start', 3))
    assert not scheduler.apply(event(plan, 'step-t2', 'complete', 2))
    for patch in ({'task_id': 'wrong'}, {'robot_ids': ['wrong']}):
        incoming = event(plan, 'step-t2', 'complete', 4)
        incoming.update(patch)
        assert not scheduler.apply(incoming)
    assert scheduler.snapshot() == active
    assert scheduler.apply(event(plan, 'step-t2', 'complete', 4))
    completed = scheduler.snapshot()
    for kind in ('start', 'complete', 'fail', 'cancel'):
        assert not scheduler.apply(event(plan, 'step-t2', kind, 5))
    assert scheduler.snapshot() == completed
    assert sum(s['state'] == S.COMPLETED for s in completed['steps']) == 1


@pytest.mark.parametrize('patch', [
    {'run_id': 'old'}, {'mission_id': 'old'}, {'step_id': []}, {'kind': {}},
    {'stamp': None}, {'stamp': {'sec': True, 'nanosec': 0}},
    {'stamp': {'sec': 1, 'nanosec': 10**9}}, {'task_id': ''},
    {'robot_ids': []}, {'robot_ids': [[]]}, {'robot_ids': ['r', 'r']},
    {'reason': {'code': 9999, 'detail': ''}}, {'reason': []},
])
def test_malformed_events_are_noops(plan, patch):
    """Reject invalid event shapes without state changes or exceptions."""
    scheduler = DependencyScheduler(plan)
    before = scheduler.snapshot()
    incoming = event(plan, 'step-t2', 'start')
    incoming.update(patch)
    assert not scheduler.apply(incoming)
    assert scheduler.snapshot() == before


def test_two_steps_same_target_are_independent():
    """Track repeated targets separately and disallow reused task IDs."""
    plan = make_plan([step('a'), step('b')])
    scheduler = DependencyScheduler(plan)
    finish(scheduler, plan, 'a')
    reused = event(plan, 'b', 'start', 3)
    reused['task_id'] = 'task-a'
    assert not scheduler.apply(reused)
    finish(scheduler, plan, 'b', 3)
    assert scheduler.snapshot()['state'] == M.COMPLETED
    assert len(scheduler.snapshot()['steps']) == 2


@pytest.mark.parametrize('case', ['unvalidated', 'empty', 'duplicate', 'unknown', 'cycle'])
def test_invalid_preconditions_fail_fast(plan, case):
    """Reject impossible graphs rather than constructing a stuck scheduler."""
    if case == 'unvalidated':
        plan['valid'] = False
    elif case == 'empty':
        plan['steps'] = []
    elif case == 'duplicate':
        plan['steps'].append(deepcopy(plan['steps'][0]))
    elif case == 'unknown':
        plan['steps'][0]['predecessor_step_ids'] = ['unknown']
    else:
        plan['steps'][0]['predecessor_step_ids'] = ['step-tc1']
    with pytest.raises(ValueError, match='PLAN_INVALID'):
        DependencyScheduler(plan)


@pytest.mark.parametrize('seed', range(200))
def test_200_seeded_dags_complete_deterministically(seed):
    """Generate shuffled DAGs and replay events, checking every prerequisite and terminal."""
    rng = random.Random(31000 + seed)
    count = rng.randint(1, 35)
    steps = [step(f's{i:02}', [f's{j:02}' for j in range(i) if rng.random() < .18],
                  rng.randrange(8)) for i in range(count)]
    rng.shuffle(steps)
    plan = make_plan(steps)
    left, right = DependencyScheduler(plan), DependencyScheduler(plan)
    completed, tick = set(), 1
    while len(completed) < count:
        ready = left.ready_steps()
        assert ready and ready == right.ready_steps()
        chosen = rng.choice(ready)
        assert set(chosen['predecessor_step_ids']) <= completed
        sid = chosen['step_id']
        for kind in ('start', 'complete'):
            incoming = event(plan, sid, kind, tick)
            assert left.apply(incoming) and right.apply(incoming)
            assert left.snapshot() == right.snapshot()
            tick += 1
        completed.add(sid)
    assert left.snapshot()['state'] == M.COMPLETED
    frozen = left.snapshot()
    assert not left.apply(event(plan, sid, 'fail', tick))
    assert left.snapshot() == frozen


def test_concurrent_roots_wait_for_all_join_predecessors():
    """An empty ready list with running work is not misclassified as terminal failure."""
    plan = make_plan([step('a'), step('b'), step('c', ['a', 'b'])])
    scheduler = DependencyScheduler(plan)
    scheduler.apply(event(plan, 'a', 'start', 1))
    scheduler.apply(event(plan, 'b', 'start', 2))
    assert not scheduler.ready_steps()
    assert scheduler.snapshot()['state'] == M.EXECUTING
    scheduler.apply(event(plan, 'b', 'complete', 3))
    assert not scheduler.ready_steps()
    scheduler.apply(event(plan, 'a', 'complete', 4))
    finish(scheduler, plan, 'c', 5)
    assert scheduler.snapshot()['state'] == M.COMPLETED
