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

"""Small-fleet assignment and conservative static-grid allocation estimates."""

import heapq
import itertools
import math


class GridCost:
    """Estimate path length with A*; execution remains exclusively P1/Nav2."""

    def __init__(self, width, height, resolution, origin, data, clearance=0.26):
        self.width, self.height, self.resolution = width, height, resolution
        self.ox, self.oy, self.yaw = origin
        self.data = data
        self.clearance = clearance
        self.blocked = set()
        self.fields = {}
        cells = math.ceil(clearance / resolution)
        offsets = [(dx, dy) for dx in range(-cells, cells + 1)
                   for dy in range(-cells, cells + 1)
                   if math.hypot(dx, dy) * resolution <= clearance + resolution * 0.71]
        for i, value in enumerate(data):
            if value < 0 or value >= 65:
                x, y = i % width, i // width
                self.blocked.update((x + dx, y + dy) for dx, dy in offsets)

    def cell(self, point):
        """Transform a map point into the possibly rotated occupancy grid."""
        dx, dy = point[0] - self.ox, point[1] - self.oy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (math.floor((c * dx + s * dy) / self.resolution),
                math.floor((-s * dx + c * dy) / self.resolution))

    def free(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height and cell not in self.blocked

    def validate_parking(self, targets, settings):
        """Check nominal stop footprints against raw occupied/unknown map cells.

        Unlike the circular allocation inflation, this respects the asymmetric
        footprint and approach heading. Nav2 still validates the actual path.
        """
        from .scheduling import entry_heading, footprint, polygon_gap
        c, s = math.cos(self.yaw), math.sin(self.yaw)

        def world(x, y):
            return (self.ox + self.resolution * (c * x - s * y),
                    self.oy + self.resolution * (s * x + c * y))

        for target in targets:
            for i, slot in enumerate(target.slots):
                poses = [(slot[0], slot[1], entry_heading(target, i))]
                if target.approaches:
                    poses.append(target.approaches[i])
                for pose in poses:
                    poly = footprint(*pose, settings)
                    cells = [self.cell(p) for p in poly]
                    padding = math.ceil(settings.parking_map_clearance_m / self.resolution) + 1
                    for x in range(min(p[0] for p in cells) - padding,
                                   max(p[0] for p in cells) + padding + 1):
                        for y in range(min(p[1] for p in cells) - padding,
                                       max(p[1] for p in cells) + padding + 1):
                            in_map = 0 <= x < self.width and 0 <= y < self.height
                            value = self.data[y * self.width + x] if in_map else -1
                            if 0 <= value < 65:
                                continue
                            cell_poly = [world(x, y), world(x + 1, y),
                                         world(x + 1, y + 1), world(x, y + 1)]
                            if polygon_gap(poly, cell_poly) <= settings.parking_map_clearance_m:
                                raise ValueError(f'{target.target_id}/{i}: stop footprint '
                                                 'too close to occupied/unknown map cell')
        return self

    def prepare(self, points):
        """Build exact distance fields off the ROS thread for fixed destinations."""
        for destination in {self.cell(point) for point in points}:
            distances = {destination: 0.0} if self.free(destination) else {}
            queue = [(0.0, destination)] if distances else []
            while queue:
                cost, current = heapq.heappop(queue)
                if cost != distances[current]:
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (1, 1), (-1, 1), (1, -1), (-1, -1)):
                    nxt = (current[0] + dx, current[1] + dy)
                    if not self.free(nxt) or (dx and dy and (
                            not self.free((current[0] + dx, current[1]))
                            or not self.free((current[0], current[1] + dy)))):
                        continue
                    proposed = cost + math.hypot(dx, dy)
                    if proposed < distances.get(nxt, math.inf):
                        distances[nxt] = proposed
                        heapq.heappush(queue, (proposed, nxt))
            self.fields[destination] = distances
        return self

    def __call__(self, start, goal):
        """Return metres or infinity for blocked/unreachable endpoints."""
        source, destination = self.cell(start), self.cell(goal)
        if not self.free(destination):
            return math.inf
        if not self.free(source):
            escaped = self._escape_inflation(source)
            if escaped is None:
                return math.inf
            source, offset = escaped
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            x, y = (source[0] + 0.5) * self.resolution, (source[1] + 0.5) * self.resolution
            point = (self.ox + c * x - s * y, self.oy + s * x + c * y)
            return offset * self.resolution + self(point, goal)
        if destination in self.fields:
            return self.fields[destination].get(source, math.inf) * self.resolution
        queue = [(0.0, 0.0, source)]
        best = {source: 0.0}
        while queue:
            _, cost, current = heapq.heappop(queue)
            if cost != best[current]:
                continue
            if current == destination:
                return cost * self.resolution
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (-1, 1), (1, -1), (-1, -1)):
                nxt = (current[0] + dx, current[1] + dy)
                if not self.free(nxt):
                    continue
                if dx and dy and (not self.free((current[0] + dx, current[1]))
                                  or not self.free((current[0], current[1] + dy))):
                    continue
                proposed = cost + math.hypot(dx, dy)
                if proposed < best.get(nxt, math.inf):
                    best[nxt] = proposed
                    heapq.heappush(queue, (proposed + math.dist(nxt, destination),
                                           proposed, nxt))
        return math.inf

    def _escape_inflation(self, source):
        """Bounded estimate for a raw-free start inside allocation inflation.

        A valid oriented robot can sit closer to a wall than the allocation
        circle. Traverse only raw-free cells, never through walls or unknowns;
        this estimates assignment cost, not permission for P1 to drive there.
        """
        def raw_free(cell):
            x, y = cell
            return (0 <= x < self.width and 0 <= y < self.height
                    and 0 <= self.data[y * self.width + x] < 65)

        if not raw_free(source):
            return None
        limit = self.clearance / self.resolution + 2
        queue, seen = [(0.0, source)], {source: 0.0}
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost != seen[cell]:
                continue
            if self.free(cell):
                return cell, cost
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (-1, 1), (1, -1), (-1, -1)):
                nxt = (cell[0] + dx, cell[1] + dy)
                proposed = cost + math.hypot(dx, dy)
                if (proposed > limit or not raw_free(nxt) or (dx and dy and (
                        not raw_free((cell[0] + dx, cell[1]))
                        or not raw_free((cell[0], cell[1] + dy))))):
                    continue
                if proposed < seen.get(nxt, math.inf):
                    seen[nxt] = proposed
                    heapq.heappush(queue, (proposed, nxt))
        return None


def allocate(robots, slots, path_cost, approaches=(), parking_tolerance=0.0, discouraged=()):
    """Minimise makespan, then total path length, with deterministic tie breaks."""
    choices = []
    for assignment in itertools.permutations(sorted(robots), len(slots)):
        costs = []
        for i, (r, slot) in enumerate(zip(assignment, slots)):
            if math.dist((robots[r].x, robots[r].y), slot[:2]) <= parking_tolerance:
                costs.append(0.0)
                continue
            first = approaches[i][:2] if approaches else slot[:2]
            cost = path_cost((robots[r].x, robots[r].y), first)
            if approaches:
                # P1 covers approach -> slot with one short straight move (forward or in
                # reverse); validate_parking checks the slot itself. A grid path would
                # wrongly fail on SLAM cells next to the stand the slot sits beside.
                cost += math.dist(first, slot[:2])
            costs.append(cost)
        if all(math.isfinite(c) for c in costs):
            penalty = sum(r in discouraged for r in assignment)
            choices.append(((penalty, max(costs), sum(costs), assignment), assignment))
    return min(choices)[1] if choices else None
