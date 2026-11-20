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
Load the repository-level competition configuration and resolve it per run mode.

The resolution rules follow the header of ``config/competition_parameters.yaml``:

* ``competition``: only ``official`` values are used. Every path listed in
  ``run_modes.required_for_motion`` must be set, otherwise motion is refused.
* ``rehearsal``: ``official`` values win; a missing (``null`` or empty) official
  value falls back to ``development`` at the same path and is recorded as a
  fallback. Required paths must be set after the fallback.
* ``development``: like ``rehearsal`` but required paths may stay unset.

Nodes should receive resolved values as ROS parameters from launch files and
must not implement their own fallback logic.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import hashlib
import os
from pathlib import Path
from typing import Any

import yaml

RUN_MODES = ('development', 'rehearsal', 'competition')
CONFIG_DIR_ENV = 'EDGEAI_CONFIG_DIR'
PARAMETERS_FILE = 'competition_parameters.yaml'
RUNTIME_FILE = 'interface_runtime.yaml'


class ConfigurationError(RuntimeError):
    """Raised when the configuration cannot be found or is inconsistent."""


class MissingOfficialParameters(ConfigurationError):
    """Raised when a run mode requires parameters that are still unset."""

    def __init__(self, run_mode: str, missing: list[str]):
        self.run_mode = run_mode
        self.missing = missing
        super().__init__(
            f'run_mode={run_mode} requires unset parameters: {", ".join(missing)}')


@dataclass
class ResolvedParameters:
    """Parameters resolved for one run mode."""

    run_mode: str
    values: dict[str, Any]
    fallbacks: list[str] = field(default_factory=list)
    missing_for_motion: list[str] = field(default_factory=list)
    sha256: str = ''

    def get(self, dotted_path: str, default: Any = None) -> Any:
        """Return the value at ``a.b.c`` or ``default`` when unset."""
        value = lookup(self.values, dotted_path)
        return default if is_unset(value) else value

    def require(self, dotted_path: str) -> Any:
        """Return the value at ``dotted_path`` or raise ConfigurationError."""
        value = lookup(self.values, dotted_path)
        if is_unset(value):
            raise ConfigurationError(
                f'parameter {dotted_path} is unset for run_mode={self.run_mode}')
        return value


def relocalization_poses(resolved: ResolvedParameters) -> dict[str, Any]:
    """
    Return the poses that Relocalize.MODE_KNOWN_START_POSE accepts, by pose ID.

    These are ``localization.start_poses`` and ``localization.task2b_start_poses``
    (the predefined Task 2B starting positions); an ID may appear in only one.
    """
    poses = dict(resolved.get('localization.start_poses', {}) or {})
    task2b = resolved.get('localization.task2b_start_poses', {}) or {}
    shared = sorted(poses.keys() & task2b.keys())
    if shared:
        raise ConfigurationError(f'pose IDs in both start_poses and task2b_start_poses: {shared}')
    poses.update(task2b)
    return poses


def find_config_dir(explicit: str | os.PathLike | None = None) -> Path:
    """
    Locate the repository ``config`` directory.

    Order: explicit argument, ``EDGEAI_CONFIG_DIR``, then the ``config``
    directory of the first parent of the current working directory that
    contains ``config/competition_parameters.yaml``.
    """
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get(CONFIG_DIR_ENV):
        candidates.append(Path(os.environ[CONFIG_DIR_ENV]))
    cwd = Path.cwd().resolve()
    candidates.extend(parent / 'config' for parent in (cwd, *cwd.parents))
    for candidate in candidates:
        if (candidate / PARAMETERS_FILE).is_file():
            return candidate.resolve()
    raise ConfigurationError(
        f'cannot find {PARAMETERS_FILE}; set {CONFIG_DIR_ENV} or the config_dir argument')


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ConfigurationError(f'{path} does not contain a mapping')
    return value


def load_runtime_contract(config_dir: str | os.PathLike | None = None) -> dict[str, Any]:
    """Return ``interface_runtime.yaml`` as a dictionary."""
    return load_yaml(find_config_dir(config_dir) / RUNTIME_FILE)


def is_unset(value: Any) -> bool:
    """Treat ``None`` and empty mappings or lists as unpublished values."""
    return value is None or (isinstance(value, (dict, list)) and not value)


def lookup(mapping: dict[str, Any], dotted_path: str) -> Any:
    value: Any = mapping
    for key in dotted_path.split('.'):
        if not isinstance(value, dict) or key not in value:
            return None
        value = value[key]
    return value


def _merge(official: Any, development: Any, path: str, fallbacks: list[str]) -> Any:
    is_branch = not _is_leaf_mapping(path) and (
        (isinstance(official, dict) and bool(official))
        or (isinstance(development, dict) and bool(development)))
    if is_branch:
        off = official if isinstance(official, dict) else {}
        dev = development if isinstance(development, dict) else {}
        return {
            key: _merge(off.get(key), dev.get(key), f'{path}.{key}' if path else str(key),
                        fallbacks)
            for key in off.keys() | dev.keys()
        }
    if not is_unset(official):
        return copy.deepcopy(official)
    if not is_unset(development):
        fallbacks.append(path)
        return copy.deepcopy(development)
    return copy.deepcopy(official)


# Mappings whose content is data (e.g. ID tables) rather than nested parameters.
_LEAF_MAPPINGS = ('aruco.id_mapping', 'localization.start_poses',
                  'localization.task2b_start_poses')


def _is_leaf_mapping(path: str) -> bool:
    return path in _LEAF_MAPPINGS


def resolve_parameters(run_mode: str,
                       config_dir: str | os.PathLike | None = None) -> ResolvedParameters:
    """
    Resolve ``competition_parameters.yaml`` for ``run_mode``.

    Raises MissingOfficialParameters for ``competition`` and ``rehearsal`` when
    a path in ``run_modes.required_for_motion`` is still unset.
    """
    if run_mode not in RUN_MODES:
        raise ConfigurationError(f'unknown run_mode {run_mode!r}; expected one of {RUN_MODES}')
    path = find_config_dir(config_dir) / PARAMETERS_FILE
    document = load_yaml(path)
    official = document.get('official') or {}
    development = document.get('development') or {}
    required = (document.get('run_modes') or {}).get('required_for_motion') or []

    fallbacks: list[str] = []
    if run_mode == 'competition':
        values = copy.deepcopy(official)
    else:
        values = _merge(official, development, '', fallbacks)

    missing = [item for item in required if is_unset(lookup(values, item))]
    if missing and run_mode != 'development':
        raise MissingOfficialParameters(run_mode, missing)

    digest = hashlib.sha256(
        yaml.safe_dump({'run_mode': run_mode, 'values': values}, sort_keys=True).encode('utf-8')
    ).hexdigest()
    return ResolvedParameters(run_mode=run_mode, values=values, fallbacks=sorted(fallbacks),
                              missing_for_motion=missing, sha256=digest)
