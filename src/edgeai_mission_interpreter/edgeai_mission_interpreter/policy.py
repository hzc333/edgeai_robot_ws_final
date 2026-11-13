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
Policy table for the four cases the rulebook and the shared interfaces leave open.

The values live in ``config/policy.yaml`` so the team can change a decision
without touching code. Unknown keys or values are rejected when loading.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from pathlib import Path
from typing import Any, Mapping

import yaml

PACKAGE_NAME = 'edgeai_mission_interpreter'

ALLOWED_VALUES = {
    'or_dependency': ('tighten_to_all', 'reject'),
    'time_limit': ('raise_priority', 'reject'),
    'named_robot': ('drop_robot', 'reject'),
    'duplicate_target': ('merge', 'keep_both'),
}


class PolicyError(ValueError):
    """The policy table is missing a key, has an unknown key or an unknown value."""


@dataclass(frozen=True)
class Policy:
    or_dependency: str
    time_limit: str
    named_robot: str
    duplicate_target: str

    def __post_init__(self):
        for f in fields(self):
            value = getattr(self, f.name)
            if value not in ALLOWED_VALUES[f.name]:
                raise PolicyError(
                    f'policy {f.name}={value!r} is not one of {ALLOWED_VALUES[f.name]}')

    def with_changes(self, **changes: str) -> 'Policy':
        return replace(self, **changes)


def default_policy_path() -> Path:
    """Return config/policy.yaml from the source tree, else from the installed share dir."""
    source = Path(__file__).resolve().parent.parent / 'config' / 'policy.yaml'
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as err:
        raise PolicyError(f'cannot find {source} and ament_index_python is missing') from err
    return Path(get_package_share_directory(PACKAGE_NAME)) / 'config' / 'policy.yaml'


def policy_from_mapping(data: Mapping[str, Any]) -> Policy:
    keys = set(data)
    expected = set(ALLOWED_VALUES)
    if keys != expected:
        raise PolicyError(
            f'policy keys differ: missing {sorted(expected - keys)}, '
            f'unknown {sorted(keys - expected)}')
    return Policy(**{k: data[k] for k in ALLOWED_VALUES})


def load_policy(path: Path | str | None = None) -> Policy:
    path = Path(path) if path is not None else default_policy_path()
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise PolicyError(f'{path}: expected a mapping')
    return policy_from_mapping(data)
