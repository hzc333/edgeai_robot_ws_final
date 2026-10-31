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

from pathlib import Path

from edgeai_common.config import (
    ConfigurationError,
    MissingOfficialParameters,
    resolve_parameters,
)
import pytest
import yaml


def _repo_config_dir():
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'config' / 'competition_parameters.yaml'
        if candidate.is_file():
            return candidate.parent
    return None


def _write(tmp_path, document):
    (tmp_path / 'competition_parameters.yaml').write_text(yaml.safe_dump(document))
    return tmp_path


SYNTHETIC = {
    'run_modes': {'required_for_motion': ['motion.max_linear_speed_mps', 'aruco.id_mapping']},
    'official': {
        'motion': {'max_linear_speed_mps': None, 'max_angular_speed_radps': 0.5},
        'aruco': {'id_mapping': {}},
    },
    'development': {
        'motion': {'max_linear_speed_mps': 0.15, 'max_angular_speed_radps': 0.3,
                   'command_timeout_s': 0.5},
        'aruco': {'id_mapping': {1: {'target_id': 'T1', 'type': 'normal'}}},
    },
}


def test_official_value_wins_and_fallbacks_are_recorded(tmp_path):
    resolved = resolve_parameters('rehearsal', _write(tmp_path, SYNTHETIC))
    assert resolved.get('motion.max_angular_speed_radps') == 0.5
    assert resolved.get('motion.max_linear_speed_mps') == 0.15
    assert resolved.get('aruco.id_mapping') == {1: {'target_id': 'T1', 'type': 'normal'}}
    assert resolved.fallbacks == [
        'aruco.id_mapping', 'motion.command_timeout_s', 'motion.max_linear_speed_mps']
    assert resolved.missing_for_motion == []
    assert len(resolved.sha256) == 64


def test_competition_uses_only_official_values(tmp_path):
    with pytest.raises(MissingOfficialParameters) as info:
        resolve_parameters('competition', _write(tmp_path, SYNTHETIC))
    assert info.value.missing == ['motion.max_linear_speed_mps', 'aruco.id_mapping']


def test_development_allows_missing_required_values(tmp_path):
    document = {
        'run_modes': {'required_for_motion': ['timing.task1_s']},
        'official': {'timing': {'task1_s': None}},
        'development': {},
    }
    resolved = resolve_parameters('development', _write(tmp_path, document))
    assert resolved.missing_for_motion == ['timing.task1_s']
    with pytest.raises(ConfigurationError):
        resolved.require('timing.task1_s')
    with pytest.raises(MissingOfficialParameters):
        resolve_parameters('rehearsal', tmp_path)


def test_unknown_run_mode_is_rejected(tmp_path):
    with pytest.raises(ConfigurationError):
        resolve_parameters('race', _write(tmp_path, SYNTHETIC))


@pytest.mark.skipif(_repo_config_dir() is None, reason='repository config not available')
def test_repository_config_resolves_per_mode():
    config_dir = _repo_config_dir()
    development = resolve_parameters('development', config_dir)
    assert development.get('motion.max_linear_speed_mps') == 0.30   # raised for the midterm check
    assert development.get('localization.start_poses')['task1_start']['x'] == -1.5
    resolve_parameters('rehearsal', config_dir)
    # Official values are still TBD, so competition mode must refuse motion.
    with pytest.raises(MissingOfficialParameters):
        resolve_parameters('competition', config_dir)
