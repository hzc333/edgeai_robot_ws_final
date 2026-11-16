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

"""Check which surfaces the camera model counts as seen and where it plans to look."""

import math

from edgeai_exploration.coverage import CameraCoverage, plan_views, surface_normal
from edgeai_exploration.frontier import Grid
import pytest


def room(width=40, height=40):
    """Return a closed square room of 0.05 m cells, origin at the outer corner."""
    rows = []
    for y in range(height):
        rows.append([100 if x in (0, width - 1) or y in (0, height - 1) else 0
                     for x in range(width)])
    return Grid(width, height, 0.05, 0.0, 0.0, 0.0, tuple(v for row in rows for v in row))


def seen_walls(grid, coverage):
    """Return the walls (west, east, south, north) with a seen surface cell."""
    walls, unseen = set(), coverage.unseen(grid)
    for y in range(grid.height):
        for x in range(grid.width):
            if surface_normal(grid, (x, y)) is not None and (x, y) not in unseen:
                walls.add('west' if x == 0 else 'east' if x == grid.width - 1
                          else 'south' if y == 0 else 'north')
    return walls


def test_surface_normals_point_into_the_room():
    grid = room()
    assert surface_normal(grid, (0, 20)) == pytest.approx((1.0, 0.0))
    assert surface_normal(grid, (20, 39)) == pytest.approx((0.0, -1.0))
    assert surface_normal(grid, (20, 20)) is None      # free cell


def test_camera_sees_only_the_wall_it_faces_within_range():
    grid = room()
    coverage = CameraCoverage(max_range_m=1.5)
    coverage.observe(grid, 1.0, 1.0, 0.0)               # facing east, wall 0.95 m ahead
    assert seen_walls(grid, coverage) == {'east'}
    far = CameraCoverage(max_range_m=0.5)
    far.observe(grid, 1.0, 1.0, 0.0)
    assert seen_walls(grid, far) == set()


def test_a_wall_seen_at_a_grazing_angle_does_not_count():
    grid = room(80, 12)
    coverage = CameraCoverage(fov_rad=math.radians(10), max_range_m=4.0,
                              max_incidence_rad=math.radians(60))
    # Looking along the corridor at 80 degrees off the north wall's normal.
    coverage.observe(grid, 0.5, 0.3, math.radians(10))
    assert 'north' not in seen_walls(grid, coverage)


def test_seen_cells_survive_a_grown_map():
    grid = room()
    coverage = CameraCoverage()
    coverage.observe(grid, 1.0, 1.0, 0.0)
    shifted = Grid(grid.width, grid.height, grid.resolution, 0.0, 0.0, 0.0, grid.data)
    assert coverage.unseen(shifted) == coverage.unseen(grid)


def test_views_look_at_the_unseen_walls():
    grid = room()
    coverage = CameraCoverage(max_range_m=2.5)
    coverage.observe(grid, 1.0, 1.0, 0.0)               # the east wall is seen
    reachable = {(x, y) for x in range(8, 32) for y in range(8, 32)}
    views = plan_views(grid, coverage, reachable, (1.0, 1.0), max_views=4)
    assert views
    headings = [math.degrees(view[2]) for view in views]
    # None of them needs to look east again; together they face the other walls.
    assert not any(abs(h) < 20 for h in headings)
    for x, y, heading, _ in views:
        coverage.observe(grid, x - coverage.offset * math.cos(heading),
                         y - coverage.offset * math.sin(heading), heading)
    assert seen_walls(grid, coverage) == {'west', 'east', 'south', 'north'}
    assert all(view[3] >= 3 for view in plan_views(grid, coverage, reachable, (1.0, 1.0)))
