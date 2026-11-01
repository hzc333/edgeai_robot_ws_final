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

"""Deterministic reachable-frontier exploration on ROS occupancy grids.

Coordinates are metres in map; yaw is radians, counterclockwise. Cell centres
are transformed by the full planar map origin, including a rotated origin.
Unknown/occupied cells are both excluded from the robot footprint.
"""

from collections import deque
from dataclasses import dataclass
import heapq
import math


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or len(self.data) != self.width * self.height:
            raise ValueError('invalid grid dimensions')
        if not all(math.isfinite(v) for v in (
                self.resolution, self.origin_x, self.origin_y, self.origin_yaw)):
            raise ValueError('non-finite grid metadata')
        object.__setattr__(self, 'origin_yaw',
                           (self.origin_yaw + math.pi) % (2 * math.pi) - math.pi)
        if self.resolution <= 0 or any(v < -1 or v > 100 for v in self.data):
            raise ValueError('invalid resolution or occupancy')

    def world(self, cell):
        x, y = (cell[0] + 0.5) * self.resolution, (cell[1] + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * x - s * y, self.origin_y + s * x + c * y

    def cell(self, x, y):
        x, y = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return math.floor((c * x + s * y) / self.resolution), math.floor(
            (-s * x + c * y) / self.resolution)

    def value(self, cell):
        x, y = cell
        return self.data[y * self.width + x] if (
            0 <= x < self.width and 0 <= y < self.height) else -1

    def neighbors(self, cell):
        x, y = cell
        for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
            other = x + dx, y + dy
            if 0 <= other[0] < self.width and 0 <= other[1] < self.height:
                yield other

    def metrics(self, reference_area_m2=None):
        area = self.resolution ** 2
        known = sum(v >= 0 for v in self.data) * area
        free = sum(0 <= v <= 25 for v in self.data) * area
        coverage = min(1.0, known / reference_area_m2) if reference_area_m2 else math.nan
        return {'known_area_m2': known, 'free_area_m2': free,
                'unknown_area_m2': sum(v < 0 for v in self.data) * area,
                'coverage_fraction': coverage}


@dataclass(frozen=True)
class Frontier:
    x: float
    y: float
    yaw: float
    cells: int
    path_m: float
    score: float


class FrontierPlanner:
    """Cluster frontiers and project onto reachable footprint-safe goal cells."""

    def __init__(self, clearance_m=0.30, minimum_cells=3, approach_m=0.8,
                 information_weight=1.0, distance_weight=1.0, goal_margin_m=0.0):
        if (clearance_m < 0 or goal_margin_m < 0
                or approach_m <= clearance_m + goal_margin_m or minimum_cells < 1):
            raise ValueError('invalid frontier parameters')
        self.clearance = clearance_m
        self.goal_clearance = clearance_m + goal_margin_m
        self.minimum_cells = minimum_cells
        self.approach = approach_m
        self.information_weight = information_weight
        self.distance_weight = distance_weight

    def candidates(self, grid, robot_xy, blacklist=()):
        free = {(x, y) for y in range(grid.height) for x in range(grid.width)
                if 0 <= grid.value((x, y)) <= 25}
        radius = math.ceil(self.clearance / grid.resolution)
        offsets = [(dx, dy) for dx in range(-radius, radius + 1)
                   for dy in range(-radius, radius + 1)
                   if math.hypot(dx, dy) * grid.resolution <= self.clearance]
        safe = {cell for cell in free if all(
            (cell[0] + dx, cell[1] + dy) in free for dx, dy in offsets)}
        goal_radius = math.ceil(self.goal_clearance / grid.resolution)
        goal_offsets = [(dx, dy) for dx in range(-goal_radius, goal_radius + 1)
                        for dy in range(-goal_radius, goal_radius + 1)
                        if math.hypot(dx, dy) * grid.resolution <= self.goal_clearance]
        goal_safe = {cell for cell in safe if all(
            (cell[0] + dx, cell[1] + dy) in free for dx, dy in goal_offsets)}
        start = grid.cell(*robot_xy)
        if start not in safe:
            return []
        distances = {start: 0.0}
        queue = [(0.0, start)]
        while queue:
            distance, cell = heapq.heappop(queue)
            if distance != distances[cell]:
                continue
            for other in grid.neighbors(cell):
                cost = distance + grid.resolution
                if other in safe and cost < distances.get(other, math.inf):
                    distances[other] = cost
                    heapq.heappush(queue, (cost, other))
        edges = {cell for cell in free if any(
            grid.value(other) == -1 for other in grid.neighbors(cell))}
        groups = []
        while edges:
            first = min(edges)
            edges.remove(first)
            group, queue = [first], deque([first])
            while queue:
                cell = queue.popleft()
                for other in grid.neighbors(cell):
                    if other in edges:
                        edges.remove(other)
                        group.append(other)
                        queue.append(other)
            if len(group) >= self.minimum_cells:
                groups.append(group)
        result = []
        reach = math.ceil(self.approach / grid.resolution)
        for group in groups:
            choices = set()
            for gx, gy in group:
                for dx in range(-reach, reach + 1):
                    for dy in range(-reach, reach + 1):
                        candidate = gx + dx, gy + dy
                        if (candidate in distances and candidate in goal_safe and
                                math.hypot(dx, dy) * grid.resolution <= self.approach):
                            choices.add(candidate)
            choices = [cell for cell in choices if not any(
                math.dist(grid.world(cell), (bx, by)) <= br for bx, by, br in blacklist)]
            if not choices:
                continue
            gx = sum(cell[0] for cell in group) / len(group)
            gy = sum(cell[1] for cell in group) / len(group)
            cell = min(choices, key=lambda c: (
                math.hypot(c[0] - gx, c[1] - gy), distances[c], c))
            x, y = grid.world(cell)
            tx, ty = grid.world((gx, gy))
            # Do not repeatedly issue a goal already occupied with the same heading.
            score = (self.information_weight * len(group) * grid.resolution
                     - self.distance_weight * distances[cell])
            result.append(Frontier(x, y, math.atan2(ty - y, tx - x), len(group),
                                   distances[cell], score))
        return sorted(result, key=lambda f: (-f.score, f.path_m, f.x, f.y))


class ExplorationProgress:
    """Bound retries and choose an explicit, reproducible termination reason."""

    def __init__(self, time_limit_s, no_progress_s, no_frontiers_s, blacklist_radius_m,
                 maximum_attempts=2):
        if min(time_limit_s, no_progress_s, no_frontiers_s, blacklist_radius_m) <= 0:
            raise ValueError('exploration limits must be positive')
        self.time_limit = time_limit_s
        self.no_progress = no_progress_s
        self.no_frontiers = no_frontiers_s
        self.radius = blacklist_radius_m
        self.maximum_attempts = maximum_attempts
        self.started = None
        self.last_growth = None
        self.empty_since = None
        self.maximum_known = 0.0
        self.failures = []

    def failed(self, x, y):
        for entry in self.failures:
            if math.dist((x, y), entry[:2]) <= self.radius:
                entry[2] += 1
                return
        self.failures.append([x, y, 1])

    def blacklist(self):
        return [(x, y, self.radius) for x, y, count in self.failures
                if count >= self.maximum_attempts]

    def update(self, elapsed, known_area, frontier_count, navigating=False):
        if self.started is None:
            self.started = self.last_growth = elapsed
        if elapsed < self.started:
            return 'clock_reset'
        if known_area > self.maximum_known + 0.01:
            self.maximum_known = known_area
            self.last_growth = elapsed
        if frontier_count or navigating:
            self.empty_since = None
        elif self.empty_since is None:
            self.empty_since = elapsed
        if elapsed - self.started >= self.time_limit:
            return 'time_limit'
        if self.empty_since is not None and elapsed - self.empty_since >= self.no_frontiers:
            return 'no_reachable_frontiers'
        if not navigating and elapsed - self.last_growth >= self.no_progress:
            return 'no_map_growth'
        return None
