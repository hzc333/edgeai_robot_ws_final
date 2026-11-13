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
Run the 60-case test set v1 (copy in test/data/testset_v1) through the interpreter.

These tests check properties every output must have. Whether each output
matches the expected answer is judged by the test set's own scorer, which
stays outside the repository (see README).
"""

import hashlib
import json
from pathlib import Path
import re

from edgeai_mission_interpreter import (
    CODE_INSTRUCTION_AMBIGUOUS,
    CODE_OUTPUT_UNSTRUCTURED,
    interpret,
)
import pytest
import yaml

DATA = Path(__file__).resolve().parent / 'data' / 'testset_v1'

# sha256 of the original files in ~/edgeai_llm_dev/testset/ when copied (2026-11-13).
ORIGINAL_SHA256 = {
    'cases.yaml': 'ffc3e496c8caf8b614d904710a30a4b69eb2a0006bcc624665611f0a1b3b2e07',
    'case.schema.json': 'd524c42a56528a801f47e9148c29251b6dbd392ff64f2ef3fdef9ee1ca3e9223',
}

# Robot ids, coordinates, paths, speeds: none may appear in any output.
FORBIDDEN = re.compile(
    r'robot[_\s]?\d|机器人\s*\d|\d\s*号机器人'
    r'|坐标|coordinate|\(\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*\)|\bx\s*[:=]|\by\s*[:=]'
    r'|路径|\bpath\b|\broute\b|waypoint|\bpose\b|位姿'
    r'|速度|\bspeed\b|velocity|m/s|米每秒|米/秒', re.I)


def _load():
    data = yaml.safe_load((DATA / 'cases.yaml').read_text('utf-8'))
    return data['registries'], data['cases']


REGISTRIES, CASES = _load()


def _results(policy):
    return [(c, interpret(c['instruction'], REGISTRIES[c['registry']], policy)) for c in CASES]


def test_copy_is_unchanged():
    for name, digest in ORIGINAL_SHA256.items():
        assert hashlib.sha256((DATA / name).read_bytes()).hexdigest() == digest, name


def test_copy_has_60_cases():
    assert len(CASES) == 60
    assert len({c['id'] for c in CASES}) == 60


def test_every_output_is_steps_or_a_failure_code(policy, step_validator):
    for case, result in _results(policy):
        if result.ok:
            step_validator(result.steps)
            assert result.issues == [], case['id']
        else:
            assert result.steps == [], case['id']
            assert [i.code for i in result.issues] in (
                [CODE_OUTPUT_UNSTRUCTURED], [CODE_INSTRUCTION_AMBIGUOUS]), case['id']


def test_no_output_mentions_robots_coordinates_paths_or_speeds(policy):
    scanned = 0
    for case, result in _results(policy):
        text = json.dumps(result.to_dict(), ensure_ascii=False)
        assert not FORBIDDEN.search(text), (case['id'], FORBIDDEN.search(text).group(0), text)
        scanned += 1
    assert scanned == 60


@pytest.mark.parametrize('sample', [
    'robot_1', 'robot 2', '机器人1', '2号机器人', '坐标', '(2, 3)', 'x: 1.0', '路径', 'path',
    'pose', '速度', 'speed', '0.5 m/s', '米每秒',
])
def test_forbidden_scan_catches_samples(sample):
    """The scan above is only meaningful if it would catch these."""
    assert FORBIDDEN.search(json.dumps({'notes': [sample]}, ensure_ascii=False))
