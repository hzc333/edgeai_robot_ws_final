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

import json
from pathlib import Path

from edgeai_mission_interpreter import load_policy, STEP_FIELDS
import jsonschema
import pytest

DEFAULT_REGISTRY = {'T1': 'normal', 'T2': 'normal', 'T3': 'normal', 'TC1': 'cooperative'}


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / 'schemas' / 'mission_plan.schema.json').is_file():
            return parent
    raise RuntimeError('repository root with schemas/mission_plan.schema.json not found')


@pytest.fixture(scope='session')
def root():
    return repo_root()


@pytest.fixture(scope='session')
def policy():
    return load_policy()


@pytest.fixture(scope='session')
def step_validator(root):
    schema = json.loads((root / 'schemas' / 'mission_plan.schema.json').read_text('utf-8'))
    step_schema = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/step',
    }
    validator = jsonschema.Draft202012Validator(step_schema)

    def check(steps):
        assert steps, 'no steps'
        for step in steps:
            assert tuple(step) == STEP_FIELDS, step
            errors = [e.message for e in validator.iter_errors(step)]
            assert not errors, (step, errors)
    return check


@pytest.fixture(scope='session')
def example_plan(root):
    path = root / 'examples' / 'interfaces' / 'mission_plan.example.json'
    return json.loads(path.read_text('utf-8'))


def relations(steps):
    """Map each step to (target, sorted predecessor targets), independent of step ids."""
    by_id = {s['step_id']: s['target_id'] for s in steps}
    return {s['target_id']: sorted(by_id[p] for p in s['predecessor_step_ids']) for s in steps}
