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

"""Validated midterm inputs and ROS-independent execution data."""

from dataclasses import dataclass, field
import math
from pathlib import Path
import re

import yaml


@dataclass(frozen=True)
class Target:
    """A completion region and separate, ordered navigation slots."""

    target_id: str
    marker_id: int
    center: tuple[float, float]
    radius: float
    required: int
    dwell: float
    slots: tuple[tuple[float, float, float], ...]
    approaches: tuple[tuple[float, float, float], ...] = ()


@dataclass(frozen=True)
class Settings:
    """Engineering limits; these are not organizer-prescribed numbers."""

    arrival_tolerance_m: float = 0.06
    state_stale_s: float = 0.5
    maximum_state_skew_s: float = 0.25
    phase_heartbeat_timeout_s: float = 2.0
    action_response_timeout_s: float = 5.0
    navigation_timeout_s: float = 150.0
    partner_wait_timeout_s: float = 180.0
    release_timeout_s: float = 5.0
    max_attempts: int = 2
    allocation_clearance_m: float = 0.26
    completion_guard_s: float = 0.3
    executor_timeout_wall_s: float = 240.0
    recovery_stable_wall_s: float = 1.0
    footprint_front_m: float = 0.115
    footprint_rear_m: float = 0.315
    footprint_half_width_m: float = 0.175
    entry_clearance_m: float = 0.05
    # Stop footprints vs raw occupied/unknown map cells (validate_parking).
    parking_map_clearance_m: float = 0.05
    holding_position_tolerance_m: float = 0.12
    holding_stable_s: float = 0.5
    retry_backoff_s: float = 1.0


@dataclass(frozen=True)
class Configuration:
    """Internal input format, independent of any LLM MissionPlan metadata."""

    robots: tuple[str, ...]
    targets: tuple[Target, ...]
    settings: Settings = field(default_factory=Settings)
    profile: str = 'kth-midterm-2026'


def load_configuration(path):
    """Reject ambiguous, malformed or accidentally altered official rules."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if (not isinstance(data, dict) or data.get('schema_version') != 1
            or data.get('rule_profile') != 'kth-midterm-2026' or data.get('frame_id') != 'map'):
        raise ValueError('expected schema 1, kth-midterm-2026, map coordinates')
    robots = tuple(data['robots'])
    if (len(robots) != 2 or len(set(robots)) != 2
            or any(not re.fullmatch(r'robot_[1-4]', r) for r in robots)):
        raise ValueError('midterm requires two distinct robot_1 .. robot_4 IDs')
    rules = {'S1': (1, (1.0, 4.1), 0.5, 1), 'C1': (2, (4.1, 1.0), 0.7, 2)}
    targets = []
    settings = Settings(**data.get('engineering', {}))
    for key, value in vars(settings).items():
        if (not isinstance(value, (int, float)) or isinstance(value, bool)
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f'engineering.{key} must be finite and positive')
    if not isinstance(settings.max_attempts, int):
        raise ValueError('max_attempts must be an integer')
    for item in data['targets']:
        tid = item['id']
        if tid not in rules:
            raise ValueError(f'unknown midterm target {tid}')
        marker, center, radius, required = rules[tid]
        if (item['marker_id'] != marker or tuple(item['center']) != center
                or item['radius_m'] != radius or item['required_robots'] != required
                or item['dwell_s'] != 10.0):
            raise ValueError(f'{tid}: official midterm rules do not match')
        slots = tuple(tuple(float(v) for v in slot) for slot in item['slots'])
        if len(slots) != required or len(set(slots)) != required:
            raise ValueError(f'{tid}: expected {required} distinct navigation slots')
        for slot in slots:
            if (len(slot) != 3 or not all(math.isfinite(v) for v in slot)
                    or math.dist(slot[:2], center) + settings.arrival_tolerance_m >= radius):
                raise ValueError(f'{tid}: slot must leave arrival margin inside target circle')
        approaches = tuple(tuple(float(v) for v in point)
                           for point in item.get('approaches', []))
        if approaches and (len(approaches) != required or any(
                len(p) != 3 or not all(math.isfinite(v) for v in p) for p in approaches)):
            raise ValueError(f'{tid}: expected one finite approach pose per slot')
        targets.append(Target(tid, marker, center, radius, required, 10.0, slots, approaches))
    if len(targets) != 2 or {t.target_id for t in targets} != set(rules):
        raise ValueError('exactly S1 and C1 are required')
    from .scheduling import validate_layout
    for target in targets:
        validate_layout(target, settings)
    return Configuration(robots, tuple(targets), settings)


@dataclass(frozen=True)
class Robot:
    """A timestamped map/base_footprint sample from one robot agent."""

    robot_id: str
    stamp: float
    x: float
    y: float
    localized: bool = True
    navigation_available: bool = True
    mode: int = 1
    active_task: str = ''
    fault: bool = False
    linear: float = 0.0
    angular: float = 0.0
    yaw: float = 0.0

    def healthy(self, now, stale):
        """Unknown/future/non-finite poses and unsafe modes cannot verify dwell."""
        return (all(math.isfinite(v) for v in (self.stamp, self.x, self.y, self.yaw))
                and -0.02 <= now - self.stamp <= stale and self.localized
                and self.navigation_available and not self.fault
                and self.mode in (1, 3, 4))

    def idle(self, now, stale):
        """Require confirmed standstill and a released action before reuse."""
        return (self.healthy(now, stale) and self.mode == 1 and not self.active_task
                and self.stationary())

    def stationary(self):
        """Measured standstill, without imposing an action mode."""
        return (math.isfinite(self.linear) and math.isfinite(self.angular)
                and abs(self.linear) <= 0.01 and abs(self.angular) <= 0.02)
