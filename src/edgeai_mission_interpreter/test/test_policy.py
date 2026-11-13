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
The four unsettled cases, each driven only by the policy table.

Every test runs the same instruction under both values of one policy entry
and checks that the result follows the table.
"""

from conftest import DEFAULT_REGISTRY, relations
from edgeai_mission_interpreter import (
    CODE_INSTRUCTION_AMBIGUOUS,
    interpret,
    load_policy,
    PolicyError,
)
from edgeai_mission_interpreter.policy import default_policy_path, policy_from_mapping
import pytest
import yaml

# 「框架」第 7 节前四条，Yueer 2026-11-13 拍板按推荐。
DECIDED_DEFAULTS = {
    'or_dependency': 'tighten_to_all',
    'time_limit': 'raise_priority',
    'named_robot': 'drop_robot',
    'duplicate_target': 'merge',
}


def _run(instruction, policy, **changes):
    return interpret(instruction, DEFAULT_REGISTRY, policy.with_changes(**changes))


def _failed_13(result):
    return not result.ok and result.steps == [] and \
        [i.code for i in result.issues] == [CODE_INSTRUCTION_AMBIGUOUS]


# --------------------------------------------------------------- policy table

def test_policy_file_holds_the_decided_defaults():
    data = yaml.safe_load(default_policy_path().read_text('utf-8'))
    assert data == DECIDED_DEFAULTS


def test_policy_rejects_unknown_value_and_missing_key():
    with pytest.raises(PolicyError):
        policy_from_mapping(dict(DECIDED_DEFAULTS, time_limit='ignore'))
    with pytest.raises(PolicyError):
        policy_from_mapping({k: v for k, v in DECIDED_DEFAULTS.items() if k != 'named_robot'})
    with pytest.raises(PolicyError):
        policy_from_mapping(dict(DECIDED_DEFAULTS, extra='x'))


def test_editing_the_policy_file_changes_the_result(tmp_path):
    """Changing the YAML file alone (no code change) flips the outcome."""
    instruction = '让 robot_1 去救援 T1。'
    edited = tmp_path / 'policy.yaml'
    edited.write_text(yaml.safe_dump(dict(DECIDED_DEFAULTS, named_robot='reject')), 'utf-8')
    before = interpret(instruction, DEFAULT_REGISTRY, load_policy())
    after = interpret(instruction, DEFAULT_REGISTRY, load_policy(edited))
    assert before.ok and relations(before.steps) == {'T1': []}
    assert _failed_13(after)


# -------------------------------------------------------------- or-dependency

def test_or_dependency_zh_operands_in_plan(policy):
    instruction = '先救援 T1 和 T3,T1 或 T3 完成后救援 TC1。'
    tight = _run(instruction, policy, or_dependency='tighten_to_all')
    assert tight.ok
    assert relations(tight.steps) == {'T1': [], 'T3': [], 'TC1': ['T1', 'T3']}
    assert _failed_13(_run(instruction, policy, or_dependency='reject'))


def test_or_dependency_en_operands_in_plan(policy):
    instruction = 'Rescue T1 and T2. After either T1 or T2 is done, rescue T3.'
    tight = _run(instruction, policy, or_dependency='tighten_to_all')
    assert tight.ok
    assert relations(tight.steps) == {'T1': [], 'T2': [], 'T3': ['T1', 'T2']}
    assert _failed_13(_run(instruction, policy, or_dependency='reject'))


@pytest.mark.parametrize('value', ['tighten_to_all', 'reject'])
def test_or_dependency_operands_not_in_plan_is_rejected(policy, value):
    """Testset tc-046: T1 and T2 are only named as the condition, so A falls back to B."""
    assert _failed_13(_run('T1 或 T2 中任意一个完成后,救援 T3。', policy, or_dependency=value))


# ----------------------------------------------------------------- time limit

def test_time_limit_zh_single_step(policy):
    instruction = '在 5 分钟内救援 T1。'
    raised = _run(instruction, policy, time_limit='raise_priority')
    assert raised.ok
    assert [(s['target_id'], s['priority']) for s in raised.steps] == [('T1', 1)]
    assert any('在 5 分钟内' in n for n in raised.notes)
    assert _failed_13(_run(instruction, policy, time_limit='reject'))


def test_time_limit_en_raises_only_the_limited_step(policy):
    instruction = 'Rescue T1 within 3 minutes, then rescue T2.'
    raised = _run(instruction, policy, time_limit='raise_priority')
    assert raised.ok
    prio = {s['target_id']: s['priority'] for s in raised.steps}
    assert prio['T1'] > prio['T2']
    assert relations(raised.steps) == {'T1': [], 'T2': ['T1']}
    assert _failed_13(_run(instruction, policy, time_limit='reject'))


def test_time_limit_goes_above_expressed_priorities(policy):
    instruction = '救援 T1、T2,其中 T2 优先级最高。3 分钟内救援 T3。'
    raised = _run(instruction, policy, time_limit='raise_priority')
    prio = {s['target_id']: s['priority'] for s in raised.steps}
    assert prio['T3'] > prio['T2'] > prio['T1']


# ---------------------------------------------------------------- named robot

def test_named_robot_zh(policy):
    instruction = '让 robot_1 去救援 T1。'
    dropped = _run(instruction, policy, named_robot='drop_robot')
    assert dropped.ok and relations(dropped.steps) == {'T1': []}
    assert _failed_13(_run(instruction, policy, named_robot='reject'))


def test_named_robot_en_keeps_order(policy):
    instruction = 'Let robot 2 rescue T3 after T1.'
    dropped = _run(instruction, policy, named_robot='drop_robot')
    assert dropped.ok and relations(dropped.steps) == {'T1': [], 'T3': ['T1']}
    assert _failed_13(_run(instruction, policy, named_robot='reject'))


def test_named_robot_number_form(policy):
    instruction = '让 2 号机器人先救援 T2,再救援 T1。'
    dropped = _run(instruction, policy, named_robot='drop_robot')
    assert dropped.ok and relations(dropped.steps) == {'T2': [], 'T1': ['T2']}
    assert _failed_13(_run(instruction, policy, named_robot='reject'))


# ------------------------------------------------------------ same target twice

def test_duplicate_target_in_sequence(policy):
    instruction = '先救援 T1,再救援 T2,最后再救援一次 T1。'
    merged = _run(instruction, policy, duplicate_target='merge')
    assert merged.ok
    assert relations(merged.steps) == {'T1': [], 'T2': ['T1']}
    both = _run(instruction, policy, duplicate_target='keep_both')
    assert both.ok
    assert [(s['step_id'], s['target_id'], s['predecessor_step_ids']) for s in both.steps] == [
        ('step-t1', 'T1', []),
        ('step-t2', 'T2', ['step-t1']),
        ('step-t1-2', 'T1', ['step-t1', 'step-t2']),
    ]


def test_duplicate_target_twice(policy):
    instruction = 'Rescue T1 twice.'
    merged = _run(instruction, policy, duplicate_target='merge')
    assert [s['step_id'] for s in merged.steps] == ['step-t1']
    both = _run(instruction, policy, duplicate_target='keep_both')
    assert [(s['step_id'], s['predecessor_step_ids']) for s in both.steps] == [
        ('step-t1', []), ('step-t1-2', ['step-t1'])]
