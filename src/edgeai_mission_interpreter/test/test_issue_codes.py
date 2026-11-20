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

"""Constants in the interpreter must equal the ones in the shared interface files."""

import json
import re

from edgeai_mission_interpreter import interpreter as interp


def _msg_constants(root, name):
    text = (root / 'src' / 'edgeai_robot_interfaces' / 'msg' / name).read_text('utf-8')
    return {m.group(2): int(m.group(3))
            for m in re.finditer(r'^\s*(uint\d+|int\d+)\s+([A-Z0-9_]+)\s*=\s*(\d+)', text, re.M)}


def test_issue_codes_match_mission_validation_issue_msg(root):
    codes = _msg_constants(root, 'MissionValidationIssue.msg')
    assert codes['CODE_OUTPUT_UNSTRUCTURED'] == interp.CODE_OUTPUT_UNSTRUCTURED == 12
    assert codes['CODE_INSTRUCTION_AMBIGUOUS'] == interp.CODE_INSTRUCTION_AMBIGUOUS == 13


def test_only_interpreter_stage_codes_are_used(root):
    codes = _msg_constants(root, 'MissionValidationIssue.msg')
    used = {v for k, v in vars(interp).items() if k.startswith('CODE_')}
    interpreter_stage = {codes['CODE_MODEL_UNAVAILABLE'], codes['CODE_MODEL_TIMEOUT'],
                         codes['CODE_OUTPUT_UNSTRUCTURED'], codes['CODE_INSTRUCTION_AMBIGUOUS']}
    assert used <= interpreter_stage


def test_action_matches_mission_step_msg_and_schema(root):
    consts = _msg_constants(root, 'MissionStep.msg')
    assert {k for k in consts if k.startswith('ACTION_')} == {'ACTION_RESCUE'}
    schema = json.loads((root / 'schemas' / 'mission_plan.schema.json').read_text('utf-8'))
    step = schema['$defs']['step']
    assert step['properties']['action'] == {'const': interp.ACTION_RESCUE}
    assert set(step['required']) == set(interp.STEP_FIELDS) == set(step['properties'])
    assert step['additionalProperties'] is False


def test_target_types_match_target_list_schema(root):
    schema = (root / 'schemas' / 'target_list.schema.json').read_text('utf-8')
    assert f'"{interp.NORMAL}"' in schema and f'"{interp.COOPERATIVE}"' in schema
