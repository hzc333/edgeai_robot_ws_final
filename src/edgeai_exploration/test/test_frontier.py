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

"""Check reachable-frontier geometry, safety and termination on synthetic maps."""

import math

from edgeai_exploration.frontier import ExplorationProgress, FrontierPlanner, Grid
import pytest


def grid_from_rows(rows, yaw=0.0):
    values = {'.': 0, '#': 100, '?': -1}
    return Grid(len(rows[0]), len(rows), 0.1, -1.0, 2.0, yaw,
                tuple(values[c] for row in rows for c in row))


def test_rotated_origin_roundtrip():
    grid = grid_from_rows(['....', '....'], math.pi / 2)
    for cell in [(0, 0), (3, 1)]:
        assert grid.cell(*grid.world(cell)) == cell
    assert math.isnan(grid.metrics()['coverage_fraction'])
    assert grid.metrics(0.08)['coverage_fraction'] == pytest.approx(1.0)


def test_frontier_goals_known_free_and_reachable():
    grid = grid_from_rows(['##########', '#......???', '#......???', '#......???',
                           '#......???', '#......???', '##########'])
    planner = FrontierPlanner(clearance_m=0.1, minimum_cells=2, approach_m=0.4)
    result = planner.candidates(grid, grid.world((2, 3)))
    assert result
    for target in result:
        cell = grid.cell(target.x, target.y)
        assert grid.value(cell) == 0
        assert all(grid.value(n) == 0 for n in grid.neighbors(cell))
        assert target.path_m > 0
    blocked = planner.candidates(grid, grid.world((2, 3)),
                                 [(f.x, f.y, 10.0) for f in result])
    assert blocked == []


def test_disconnected_frontier_and_unsafe_start_rejected():
    grid = grid_from_rows(['############', '#...#....???', '#...#....???',
                           '#...#....???', '#...#....???', '############'])
    planner = FrontierPlanner(clearance_m=0.1, minimum_cells=2, approach_m=0.3)
    assert planner.candidates(grid, grid.world((2, 2))) == []
    assert planner.candidates(grid, grid.world((0, 0))) == []


def test_retry_blacklist_and_explicit_end_reasons():
    progress = ExplorationProgress(100, 20, 5, 0.3)
    progress.failed(1, 1)
    assert not progress.blacklist()
    progress.failed(1.1, 1)
    assert progress.blacklist() == [(1, 1, 0.3)]
    assert progress.update(0, 1, 2) is None
    assert progress.update(2, 2, 0) is None
    assert progress.update(8, 2, 0) == 'no_reachable_frontiers'
    assert progress.update(100, 2, 1, navigating=True) == 'time_limit'


def test_invalid_map_rejected():
    with pytest.raises(ValueError):
        Grid(2, 2, 0.05, 0, 0, 0, (0, 0))
    with pytest.raises(ValueError):
        Grid(1, 1, float('nan'), 0, 0, 0, (0,))


def test_goal_margin_preserves_clearance_after_arrival_error():
    grid = grid_from_rows(['####################'] + ['#..............?????'] * 12
                          + ['####################'])
    planner = FrontierPlanner(clearance_m=0.1, minimum_cells=2, approach_m=0.6,
                              goal_margin_m=0.2)
    # The current cell need only meet traversal clearance, not extra goal clearance.
    goals = planner.candidates(grid, grid.world((2, 2)))
    assert goals
    for goal in goals:
        cell = grid.cell(goal.x, goal.y)
        for y in range(grid.height):
            for x in range(grid.width):
                if math.dist((x, y), cell) * grid.resolution <= 0.3:
                    assert grid.value((x, y)) == 0
