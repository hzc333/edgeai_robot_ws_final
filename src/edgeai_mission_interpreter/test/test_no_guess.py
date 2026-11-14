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
Card 01b: do not guess.

The phrasings here were written for these tests. None is taken from the
held-out set (the author never read it) or copied from a dev case.
"""

from pathlib import Path
import re

from conftest import DEFAULT_REGISTRY, relations
from edgeai_mission_interpreter import CODE_INSTRUCTION_AMBIGUOUS, interpret
import pytest

LARGE = dict(DEFAULT_REGISTRY, T4='normal', T5='normal', TC2='cooperative')


def _refused(result):
    return (not result.ok and result.steps == []
            and [i.code for i in result.issues] == [CODE_INSTRUCTION_AMBIGUOUS])


def _prio(result):
    return {s['target_id']: s['priority'] for s in result.steps}


# ------------------------------------------------- unknown constraint words

@pytest.mark.parametrize('instruction', [
    'Rescue T1 while T2 is being rescued.',
    'Keep rescuing T1 until T3 is done.',
    'Rescue T2 prior to T1.',
    'Rescue T3, followed by T1.',
    'Start with T2 and end with T1.',
    'Rescue T1 and T2; T2 is critical.',
    'Rescue T1, T2 and T3, each one separately.',
    'Rescue T2 sooner than T3.',
    'Rescue T1 immediately, T2 later.',
    'Rescue T1, and only then T2.',
    'T1 晚于 T2 救援。',
    '救援 T1 和 T2，T2 优先。',
    '救援 T1，T2 等下再救。',
    '救援T1，T3随即救援。',
])
def test_unknown_constraint_word_is_refused(instruction, policy):
    result = interpret(instruction, DEFAULT_REGISTRY, policy)
    assert _refused(result), result
    assert '没有被任何规则用上' in result.issues[0].detail


def test_no_order_statement_contradicting_the_plan_is_refused(policy):
    result = interpret('First rescue T1, then T2. T1 and T2 can be rescued in any order.',
                       DEFAULT_REGISTRY, policy)
    assert _refused(result)


def test_last_contradicting_another_order_is_refused(policy):
    assert _refused(interpret('Rescue T1 last, then rescue T2.', DEFAULT_REGISTRY, policy))


def test_unmarked_clause_after_first_is_refused(policy):
    assert _refused(interpret('先救援 T1，T2 和 T3 同时救援。', DEFAULT_REGISTRY, policy))


# --------------------------------------------------------------- new rules

@pytest.mark.parametrize('instruction, registry, expected', [
    # 等 ... 完成后 (the subject waits), and 「也要」 only adds a target
    ('T3要等T1完成后再救援，T1也要救援。', DEFAULT_REGISTRY, {'T3': ['T1'], 'T1': []}),
    # 等 without a subject: it refers to the clause before
    ('TC1要协同救援，但必须等T2救完之后；T2也要救援。', DEFAULT_REGISTRY,
     {'TC1': ['T2'], 'T2': []}),
    # 前提是 ... 都已完成, plus a no-order statement with 之间
    ('救援T3的前提是T1和T2都已完成，T1、T2之间不分先后。', DEFAULT_REGISTRY,
     {'T3': ['T1', 'T2'], 'T1': [], 'T2': []}),
    # X 之前必须先 Y, without 在
    ('救援T2之前必须先救援T3。', DEFAULT_REGISTRY, {'T2': ['T3'], 'T3': []}),
    # references: 两个都 / 这三个全部
    ('先救援T1和T2，两个都完成后救援T3。', DEFAULT_REGISTRY,
     {'T1': [], 'T2': [], 'T3': ['T1', 'T2']}),
    ('首先救援T1、T2和T3，这三个全部完成后再救援TC1。', DEFAULT_REGISTRY,
     {'T1': [], 'T2': [], 'T3': [], 'TC1': ['T1', 'T2', 'T3']}),
    # count word after a list, and a condition that is only 全部
    ('T1、T2、T3三个目标先后都可以，全部救完之后救援TC1。', DEFAULT_REGISTRY,
     {'T1': [], 'T2': [], 'T3': [], 'TC1': ['T1', 'T2', 'T3']}),
    # a condition that ends the clause without 后
    ('先救援T1。T1完成后救援T2和T3。T2和T3都完成，救援TC1。', DEFAULT_REGISTRY,
     {'T1': [], 'T2': ['T1'], 'T3': ['T1'], 'TC1': ['T2', 'T3']}),
    # wait for, between ... no order
    ('Rescue T3, T1 and T2. T1 and T2 both have to wait for T3; between T1 and T2, '
     'no order is required.', DEFAULT_REGISTRY, {'T3': [], 'T1': ['T3'], 'T2': ['T3']}),
    ('Rescue T4, T1, T2 and T5. T1 and T2 must wait for T4, and T5 must wait for both T1 '
     'and T2.', LARGE, {'T4': [], 'T1': ['T4'], 'T2': ['T4'], 'T5': ['T1', 'T2']}),
    # last, a trailing condition across a list, and a reference to that list
    ('Rescue T1 last, after T2 and T3 are done; those two can be rescued in any order.',
     DEFAULT_REGISTRY, {'T1': ['T2', 'T3'], 'T2': [], 'T3': []}),
    # only after + reference
    ('Rescue T2 and T3 in any order, and rescue T1 only after both are done.',
     DEFAULT_REGISTRY, {'T2': [], 'T3': [], 'T1': ['T2', 'T3']}),
    # initially / after that
    ('Initially rescue T3, after that rescue T2.', DEFAULT_REGISTRY, {'T3': [], 'T2': ['T3']}),
    # in exactly this order
    ('Rescue T3, T1 and T2 in exactly this order.', DEFAULT_REGISTRY,
     {'T3': [], 'T1': ['T3'], 'T2': ['T1']}),
    # only ... nothing else / 只需要 ... 这一个目标
    ('Only rescue T3, nothing else.', DEFAULT_REGISTRY, {'T3': []}),
    ('只需要救援T2这一个目标。', DEFAULT_REGISTRY, {'T2': []}),
    # the same target named again without a repeat cue is the same step
    ('Rescue T1, T2 and T3. After T1 is done, rescue T2 and T3.', DEFAULT_REGISTRY,
     {'T1': [], 'T2': ['T1'], 'T3': ['T1']}),
])
def test_new_rule(instruction, registry, expected, policy, step_validator):
    result = interpret(instruction, registry, policy)
    assert result.ok, result
    step_validator(result.steps)
    assert relations(result.steps) == expected
    assert result.notes == []


def test_priority_pair_in_one_clause(policy):
    result = interpret('Rescue T1 and T3. T3 has the highest priority and T1 has a lower '
                       'priority. The order does not matter.', DEFAULT_REGISTRY, policy)
    assert _prio(result)['T3'] > _prio(result)['T1']


def test_priority_low_and_equal(policy):
    result = interpret('救援T1、T2、T3，T1优先级较低，T2和T3一样重要，不要求顺序。',
                       DEFAULT_REGISTRY, policy)
    prio = _prio(result)
    assert prio['T2'] == prio['T3'] > prio['T1']


def test_priority_ranking(policy):
    for instruction in ('救援T1、T2、T3，优先级从高到低是T3、T1、T2，不要求顺序。',
                        'Rescue T1, T2 and T3. Priority from high to low: T3, T1, T2. '
                        'No order is required.'):
        prio = _prio(interpret(instruction, DEFAULT_REGISTRY, policy))
        assert prio['T3'] > prio['T1'] > prio['T2'], instruction


# ------------------------------------------------------- no dev case ids

def test_no_dev_case_ids_in_the_package():
    """Rules are written per wording, never per case: no dev case id (numbers 101 to 599)."""
    package = Path(__file__).resolve().parents[1]
    dev_id = re.compile(r'tc-([1-5]\d\d)')
    found = []
    for path in package.rglob('*'):
        if path.is_file() and path.suffix in ('.py', '.yaml', '.md', '.xml', '.cfg'):
            if 'testset_v1' in path.parts:
                continue
            for m in dev_id.finditer(path.read_text('utf-8')):
                found.append(f'{path.name}: {m.group(0)}')
    assert found == []
