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

"""Camera coverage of the map's surfaces, and viewpoints for the surfaces not seen yet.

Frontier exploration completes the LiDAR map, not the camera's view: a marker on a wall
that robot_1 only drove past sideways never enters the 60 degree camera image (practice
arena seed 5). A surface cell (occupied, with a free neighbour) counts as seen once a
camera ray reached it within max_range_m and hit it at most max_incidence_rad off its
normal: a marker must be seen roughly face-on to be read. Cells are remembered by their
map position, so they stay seen while SLAM grows or shifts the grid.

After the frontier search, plan_views picks poses on reachable goal cells that see the
most unseen surface, greedily, like next-best-view planners for visual search.
"""

import math


def surface_normal(grid, cell):
    """Return the unit vector (map) from an occupied cell to its free neighbours, or None."""
    if grid.value(cell) <= 25:
        return None
    sx = sy = 0.0
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if (dx or dy) and 0 <= grid.value((cell[0] + dx, cell[1] + dy)) <= 25:
                sx, sy = sx + dx, sy + dy
    if not any(0 <= grid.value(other) <= 25 for other in grid.neighbors(cell)):
        return None     # inside a wall or touching free space only at a corner
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    nx, ny = c * sx - s * sy, s * sx + c * sy
    length = math.hypot(nx, ny)
    # A one-cell wall with free space on both sides has no single normal: any side counts.
    return (nx / length, ny / length) if length > 1e-6 else (0.0, 0.0)


def cell_key(grid, cell):
    x, y = grid.world(cell)
    return round(x / grid.resolution), round(y / grid.resolution)


def cast(grid, x, y, angle, max_range):
    """First occupied cell along a ray within max_range, or None (unknown or nothing)."""
    step = grid.resolution / 2
    dx, dy = math.cos(angle) * step, math.sin(angle) * step
    for index in range(1, int(max_range / step) + 1):
        cell = grid.cell(x + dx * index, y + dy * index)
        value = grid.value(cell)
        if value > 25:
            return cell
        if value < 0:
            return None
    return None


class CameraCoverage:
    """Surface cells the camera has seen closely and squarely enough to read a marker."""

    def __init__(self, fov_rad=math.radians(50), max_range_m=2.0,
                 max_incidence_rad=math.radians(60), offset_m=0.0875,
                 ray_step_rad=math.radians(1.0)):
        if not (0 < fov_rad < math.pi and max_range_m > 0 and 0 < max_incidence_rad
                <= math.pi / 2 and ray_step_rad > 0):
            raise ValueError('invalid camera coverage parameters')
        self.fov = fov_rad
        self.range = max_range_m
        self.incidence = max_incidence_rad
        self.offset = offset_m
        self.ray_step = ray_step_rad
        self.seen = set()

    def readable(self, grid, cell, ray_angle):
        """Return True if a marker on this surface cell faces a ray at ray_angle closely enough."""
        normal = surface_normal(grid, cell)
        if normal is None:
            return False
        if normal == (0.0, 0.0):
            return True
        # The marker faces back along the ray: compare the normal with the reversed ray.
        cosine = -(normal[0] * math.cos(ray_angle) + normal[1] * math.sin(ray_angle))
        return cosine >= math.cos(self.incidence)

    def observe(self, grid, x, y, yaw):
        """Record what the camera of a robot at (x, y, yaw) in map sees now."""
        cx, cy = x + self.offset * math.cos(yaw), y + self.offset * math.sin(yaw)
        rays = max(1, int(self.fov / self.ray_step))
        for index in range(rays + 1):
            angle = yaw - self.fov / 2 + index * self.fov / rays
            cell = cast(grid, cx, cy, angle, self.range)
            if cell is not None and self.readable(grid, cell, angle):
                self.seen.add(cell_key(grid, cell))

    def unseen(self, grid):
        """Surface cells of the current map the camera has not seen yet."""
        return {cell for cell in ((x, y) for y in range(grid.height) for x in range(grid.width))
                if grid.value(cell) > 25 and surface_normal(grid, cell) is not None
                and cell_key(grid, cell) not in self.seen}


def plan_views(grid, coverage, reachable, robot_xy, spacing_m=0.3, min_cells=3,
               max_views=10, ray_step_rad=math.radians(2.0)):
    """
    Poses (x, y, yaw, cells) that see the unseen surface, nearest first.

    reachable: goal cells the robot can reach (FrontierPlanner.reachable). Candidates lie
    on a spacing_m lattice of them; each takes the heading whose field of view holds the
    most unseen cells it can read. Views are picked greedily by newly seen cells until
    one adds fewer than min_cells or max_views are chosen.
    """
    targets = coverage.unseen(grid)
    if not targets or not reachable:
        return []
    step = max(1, round(spacing_m / grid.resolution))
    rays = int(2 * math.pi / ray_step_rad)
    options = []
    for cell in reachable:
        if cell[0] % step or cell[1] % step:
            continue
        x, y = grid.world(cell)
        hits = []
        for index in range(rays):
            angle = -math.pi + index * ray_step_rad
            hit = cast(grid, x, y, angle, coverage.range)
            if hit in targets and coverage.readable(grid, hit, angle):
                hits.append((angle, hit))
        if not hits:
            continue
        # Best heading: the window of the field of view holding the most distinct cells.
        best = set()
        for centre, _ in hits:
            window = {hit for angle, hit in hits
                      if abs(math.atan2(math.sin(angle - centre), math.cos(angle - centre)))
                      <= coverage.fov / 2}
            if len(window) > len(best):
                best, heading = window, centre
        options.append((x, y, heading, best))
    chosen, covered = [], set()
    while len(chosen) < max_views and options:
        x, y, heading, cells = max(options, key=lambda o: (len(o[3] - covered), -o[0], -o[1]))
        new = cells - covered
        if len(new) < min_cells:
            break
        chosen.append((x, y, heading, len(new)))
        covered |= new
    ordered, here = [], robot_xy
    while chosen:
        view = min(chosen, key=lambda v: math.dist(here, v[:2]))
        chosen.remove(view)
        ordered.append(view)
        here = view[:2]
    return ordered
