"""Behavior and repository-contract tests for card 02."""

import copy
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

import pytest
from jsonschema import Draft202012Validator

from edgeai_rescue_coordinator import validator as v

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads((ROOT / path).read_text())


def candidate_from_log(log):
    """Explicit test-only adapter; log schema is NOT the candidate schema."""
    return {
        'header': {'stamp': {'sec': 0, 'nanosec': 0}, 'frame_id': ''},
        **{key: log[key] for key in ('run_id', 'mission_id', 'map_id',
                                    'map_revision', 'target_revision',
                                    'original_instruction')},
        'model_name': log['model']['name'],
        'model_version': log['model']['version'],
        'prompt_version': log['model']['prompt_version'],
        'steps': [dict(step_id=s['step_id'], target_id=s['target_id'],
                       action_type=v.ACTION_RESCUE, priority=s['priority'],
                       predecessor_step_ids=list(s['predecessor_step_ids']))
                  for s in log['steps']],
        'valid': False,
    }


@pytest.fixture
def inputs():
    plan = candidate_from_log(read('examples/interfaces/mission_plan.example.json'))
    targets = json.loads((PACKAGE / 'test/fixtures/task1_targets.complete.example.json').read_text())
    return dict(candidate=plan, frozen_targets=targets, available_robots=2,
                run_id=plan['run_id'], map_id=plan['map_id'],
                map_revision=plan['map_revision'], target_revision=plan['target_revision'],
                phase=v.PHASE_TASK2B)


def codes(inputs):
    return {issue['code'] for issue in v.validate_mission(**inputs)[1]}


def break_code(inputs, code):
    plan = inputs['candidate']
    if code == 100:
        plan['steps'][0]['priority'] = -1
    elif code == 101:
        plan['steps'].append(copy.deepcopy(plan['steps'][0]))
    elif code == 102:
        plan['steps'][0]['target_id'] = 'T9'
    elif code == 104:
        plan['steps'][0]['predecessor_step_ids'] = ['missing']
    elif code == 105:
        plan['steps'][0]['predecessor_step_ids'] = ['step-tc1']
    elif code == 106:
        inputs['available_robots'] = 1
    elif code == 107:
        plan['steps'][0]['robot_id'] = 'robot_1'
    elif code == 108:
        plan['map_revision'] += 1
    elif code == 109:
        plan['run_id'] = 'different-run'
    elif code == 110:
        inputs['phase'] = 0


@pytest.mark.parametrize('code', [100, 101, 102, 104, 105, 106, 107, 108, 109, 110])
def test_each_reachable_code_triggers(inputs, code):
    break_code(inputs, code)
    accepted, issues = v.validate_mission(**inputs)
    assert not accepted
    assert code in {i['code'] for i in issues}
    assert all(set(i) == {'code', 'step_id', 'target_id', 'detail'} for i in issues)


@pytest.mark.parametrize('code', range(100, 111))
def test_each_code_absent_on_valid_input(inputs, code):
    assert code not in codes(inputs)
    assert v.validate_mission(**inputs) == (True, [])


def test_103_has_no_candidate_type_to_compare(inputs):
    assert 'target_type' not in v._CANDIDATE.schema['properties']['steps']['items']['properties']
    inputs['candidate']['steps'][0]['target_type'] = 'cooperative'
    assert 100 in codes(inputs)  # Reject extension; do not invent a 103 comparison.
    assert 103 not in codes(inputs)


@pytest.mark.skip(reason='待队里澄清: MissionStep has no target_type; code 103 cannot be triggered')
def test_103_trigger_pending_interface_clarification():
    pass


def constants(name):
    text = (ROOT / 'src/edgeai_robot_interfaces/msg' / name).read_text()
    return {key: int(value) for key, value in re.findall(r'^uint\d+ (\w+)=(\d+)$', text, re.M)}


def fields(name):
    text = (ROOT / 'src/edgeai_robot_interfaces/msg' / name).read_text()
    return {line.split()[1]: line.split()[0] for line in text.splitlines()
            if line and not line.startswith('#') and '=' not in line}


def test_constants_and_message_fields_match_repository():
    expected = {k: n for k, n in constants('MissionValidationIssue.msg').items() if 100 <= n <= 110}
    assert len(expected) == 11
    assert {k: getattr(v, k) for k in expected} == expected
    assert v.ACTION_RESCUE == constants('MissionStep.msg')['ACTION_RESCUE']
    assert v.PHASE_TASK2B == constants('CompetitionState.msg')['PHASE_TASK2B']
    schema = v._CANDIDATE.schema
    assert set(schema['properties']) == set(fields('MissionPlan.msg'))
    step_schema = schema['properties']['steps']['items']
    assert set(step_schema['properties']) == set(fields('MissionStep.msg'))
    assert step_schema['properties']['action_type']['const'] == v.ACTION_RESCUE
    assert fields('MissionStep.msg')['priority'] == 'uint32'
    assert step_schema['properties']['priority']['maximum'] == 2**32-1
    assert (PACKAGE / 'edgeai_rescue_coordinator/schemas/target_list.schema.json').read_bytes() == (ROOT / 'schemas/target_list.schema.json').read_bytes()


@pytest.mark.parametrize('invalid', [False, True])
def test_all_inputs_and_steps_unchanged(inputs, invalid):
    if invalid:
        break_code(inputs, 105)
        break_code(inputs, 107)
    before = copy.deepcopy(inputs)
    v.validate_mission(**inputs)
    assert inputs == before


def test_valid_true_rejected_with_schema_invalid(inputs):
    inputs['candidate']['valid'] = True
    assert codes(inputs) == {100}
    assert not v.validate_mission(**inputs)[0]


def test_repository_plan_steps_accepted_with_complete_fixture(inputs):
    assert len(inputs['candidate']['steps']) == 4
    assert v.validate_mission(**inputs) == (True, [])


def test_original_registry_rejects_missing_t2_and_t3(inputs):
    inputs['frozen_targets'] = read('examples/interfaces/task1_targets.example.json')
    accepted, issues = v.validate_mission(**inputs)
    assert not accepted
    assert {(i['code'], i['target_id']) for i in issues} == {(102, 'T2'), (102, 'T3')}


def test_repository_rejected_example_t9_returns_102(inputs):
    rejected = read('examples/interfaces/mission_intake_rejected.example.json')
    log = read('examples/interfaces/mission_plan.example.json')
    log.update(steps=rejected['candidate_steps'], mission_id=rejected['mission_id'],
               original_instruction=rejected['original_instruction'])
    inputs['candidate'] = candidate_from_log(log)
    accepted, issues = v.validate_mission(**inputs)
    assert not accepted
    assert [i['code'] for i in issues] == [102]
    assert issues[0]['target_id'] == 'T9'


def test_multiple_independent_errors_returned_together(inputs):
    for code in (100, 101, 102, 104, 106, 107, 108, 109, 110):
        break_code(inputs, code)
    # Add a self-cycle without overwriting the unknown-predecessor case.
    inputs['candidate']['steps'][1]['predecessor_step_ids'] = ['step-t1']
    assert codes(inputs) == {100, 101, 102, 104, 105, 106, 107, 108, 109, 110}


def test_target_fixture_preserves_full_schema_and_original_records(inputs):
    original = read('examples/interfaces/task1_targets.example.json')
    fixture = inputs['frozen_targets']
    Draft202012Validator(read('schemas/target_list.schema.json')).validate(fixture)
    assert fixture['targets'][:2] == original['targets']
    assert {k: val for k, val in fixture.items() if k != 'targets'} == {k: val for k, val in original.items() if k != 'targets'}
    assert [t['target_id'] for t in fixture['targets'][2:]] == ['T2', 'T3']


@pytest.mark.parametrize('value', [None, [], 'bad', 1, True, {}])
def test_malformed_top_level_rejected_without_crash(inputs, value):
    inputs['candidate'] = value
    assert 100 in codes(inputs)


@pytest.mark.parametrize('value', [{1: 'bad', 'mixed': True}, {'extra': float('nan')},
                                 {'extra': float('inf')}, {'extra': object()}])
def test_non_json_data_rejected_without_crash(inputs, value):
    inputs['candidate'] = value
    assert 100 in codes(inputs)


def test_cyclic_python_container_rejected_without_hanging(inputs):
    inputs['candidate']['extra'] = inputs['candidate']
    assert 100 in codes(inputs)


@pytest.mark.parametrize('field,value', [
    ('steps', None), ('steps', [None, 5, 'bad']),
    ('steps', [{'step_id': [], 'target_id': {}, 'predecessor_step_ids': [None, {}]}]),
    ('valid', 0), ('map_revision', True), ('map_revision', 1.0),
    ('map_revision', 2**32), ('header', {}), ('model_name', ''),
])
def test_malformed_candidate_fields_rejected(inputs, field, value):
    inputs['candidate'][field] = value
    assert 100 in codes(inputs)


@pytest.mark.parametrize('field,value', [
    ('priority', True), ('priority', 1.0), ('priority', 2**32),
    ('action_type', True), ('action_type', 2),
    ('predecessor_step_ids', ['step-t2', 'step-t2']),
])
def test_step_types_and_ranges_rejected(inputs, field, value):
    inputs['candidate']['steps'][1][field] = value
    assert 100 in codes(inputs)


@pytest.mark.parametrize('field', ['robot_id', 'velocity', 'path', 'pose', 'coordinates'])
def test_structured_forbidden_fields_rejected(inputs, field):
    inputs['candidate']['steps'][0]['extra'] = {field: 'untrusted'}
    assert {100, 107} <= codes(inputs)


def test_original_instruction_is_not_keyword_scanned(inputs):
    inputs['candidate']['original_instruction'] = '让 robot_2 救 T1，不要输出速度、路径、位姿。'
    assert v.validate_mission(**inputs) == (True, [])


@pytest.mark.parametrize('robots,cooperative,accepted', [(0, False, False), (1, False, True), (1, True, False), (2, True, True)])
def test_robot_count_boundary(inputs, robots, cooperative, accepted):
    step = inputs['candidate']['steps'][3 if cooperative else 0]
    step['predecessor_step_ids'] = []
    inputs['candidate']['steps'] = [step]
    inputs['available_robots'] = robots
    assert v.validate_mission(**inputs)[0] is accepted


def test_independent_steps_can_run_sequentially_on_one_robot(inputs):
    inputs['candidate']['steps'] = inputs['candidate']['steps'][:3]
    inputs['available_robots'] = 1
    assert v.validate_mission(**inputs) == (True, [])


@pytest.mark.parametrize('phase', [0, 1, 2, 3, 4, 6, 7, 255, True, None])
def test_only_task2b_phase_allowed(inputs, phase):
    inputs['phase'] = phase
    assert 110 in codes(inputs)


@pytest.mark.parametrize('field', ['map_id', 'map_revision', 'target_revision'])
def test_each_candidate_revision_checked(inputs, field):
    inputs['candidate'][field] = 'other-map' if field == 'map_id' else 99
    assert 108 in codes(inputs)


def test_frozen_registry_run_and_revision_checked(inputs):
    inputs['frozen_targets']['run_id'] = 'old-run'
    inputs['frozen_targets']['revision'] = 99
    inputs['frozen_targets']['map']['revision'] = 99
    assert codes(inputs) == {108, 109}


@pytest.mark.parametrize('value', [None, [], {}, {'targets': [None, {}, {'target_id': []}]}])
def test_malformed_registry_rejected(inputs, value):
    inputs['frozen_targets'] = value
    assert 100 in codes(inputs)


@pytest.mark.parametrize('kind', [[], {}, None, 'unknown'])
def test_invalid_registry_target_type_rejected_without_crash(inputs, kind):
    inputs['frozen_targets']['targets'][0]['target_type'] = kind
    assert 100 in codes(inputs)
    assert 103 not in codes(inputs)


def test_duplicate_registry_ids_rejected(inputs):
    inputs['frozen_targets']['targets'].append(copy.deepcopy(inputs['frozen_targets']['targets'][0]))
    assert 100 in codes(inputs)


@pytest.mark.parametrize('field,value', [('available_robots', -1), ('available_robots', True), ('available_robots', 2.0), ('run_id', ''), ('map_id', None), ('map_revision', False), ('target_revision', -1)])
def test_invalid_authoritative_context_rejected(inputs, field, value):
    inputs[field] = value
    assert 100 in codes(inputs)


def test_long_chain_and_cycle_without_recursion_failure(inputs):
    template = inputs['candidate']['steps'][0]
    inputs['candidate']['steps'] = [dict(template, step_id=f's{i}', predecessor_step_ids=[f's{i-1}'] if i else []) for i in range(1500)]
    assert v.validate_mission(**inputs) == (True, [])
    inputs['candidate']['steps'][0]['predecessor_step_ids'] = ['s1499']
    assert codes(inputs) == {105}


def test_deterministic_results_across_hash_seeds(inputs):
    for code in (101, 104, 107, 108):
        break_code(inputs, code)
    script = 'import json,sys; from edgeai_rescue_coordinator import validate_mission; print(json.dumps(validate_mission(**json.load(sys.stdin)),sort_keys=True))'
    results = [subprocess.check_output([sys.executable, '-c', script], input=json.dumps(inputs), text=True,
               env=dict(os.environ, PYTHONHASHSEED=str(seed), PYTHONPATH=str(PACKAGE))) for seed in (1, 42, 999)]
    assert len(set(results)) == 1


def test_no_network_or_model_and_timing(inputs, monkeypatch):
    import socket

    def blocked(*args, **kwargs):
        raise AssertionError('network access attempted')

    monkeypatch.setattr(socket, 'socket', blocked)
    monkeypatch.setattr(socket, 'create_connection', blocked)
    results, durations = [], []
    for _ in range(100):
        start = time.perf_counter_ns()
        results.append(v.validate_mission(**inputs))
        durations.append((time.perf_counter_ns() - start) / 1e6)
    assert all(result == (True, []) for result in results)
    runtime = (ROOT / 'config/interface_runtime.yaml').read_text()
    timeout_s = float(re.search(r'^  validate_mission: ([0-9.]+)$', runtime, re.M)[1])
    print(f'\nTIMING calls=100 steps=4 targets=4 min_ms={min(durations):.3f} mean_ms={sum(durations)/100:.3f} max_ms={max(durations):.3f} timeout_ms={timeout_s*1000:.1f}')
    assert max(durations) < timeout_s * 1000
