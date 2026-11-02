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

"""Frozen publication and durable snapshot failure regression checks."""

import json
import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_exploration.frontier import Grid
from edgeai_exploration.map_manager import (
    add_scan_hits, center_karto_cells, clear_discs, complete_max_edges, MapManager)
from edgeai_robot_interfaces.msg import CompetitionState
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger


def test_clear_discs_frees_only_cells_near_the_robots():
    # 10 x 10 cells of 0.1 m, all occupied; a parked robot at (0.55, 0.55).
    grid = Grid(10, 10, 0.1, 0.0, 0.0, 0.0, [100] * 100)
    cleared = clear_discs(grid, [(0.55, 0.55)], 0.15)
    free = {(i % 10, i // 10) for i, v in enumerate(cleared.data) if v == 0}
    assert (5, 5) in free and (4, 5) in free and (5, 6) in free
    assert (3, 5) not in free and (0, 0) not in free and (9, 9) not in free
    assert grid.data[55] == 100        # the input grid is unchanged


def test_center_karto_cells_moves_the_origin_back_half_a_cell():
    # Karto puts x = 0.24 in cell round(2.4) = 2, centred on 0.2: after the correction
    # cell 2 spans 0.15..0.25 as in ROS.
    grid = center_karto_cells(Grid(4, 3, 0.1, 0.0, 1.0, 0.0, (0,) * 12))
    assert (grid.origin_x, grid.origin_y) == (-0.05, 0.95)
    assert math.floor((0.24 - grid.origin_x) / grid.resolution) == round(0.24 / 0.1)
    # A grid turned by 90 degrees moves along its own axes.
    turned = center_karto_cells(Grid(4, 3, 0.1, 0.0, 0.0, math.pi / 2, (0,) * 12))
    assert turned.origin_x == pytest.approx(0.05) and turned.origin_y == pytest.approx(-0.05)


def _wall_hits(points, repeat=5):
    hits = {}
    for _ in range(repeat):
        add_scan_hits(hits, points, 0.01)
    return hits


def test_complete_max_edges_restores_a_dropped_far_column():
    # 4 x 3 free cells of 0.1 m; the east wall at x = 0.41 fell one cell past the grid.
    grid = Grid(4, 3, 0.1, 0.0, 0.0, 0.0, (0,) * 12)
    done = complete_max_edges(grid, _wall_hits([(0.41, y) for y in (0.05, 0.15, 0.25)]),
                              0.01, 3)
    assert (done.width, done.height) == (5, 3)
    assert [done.data[j * 5 + 4] for j in range(3)] == [100, 100, 100]
    assert all(done.data[j * 5 + i] == 0 for j in range(3) for i in range(4))
    assert grid.width == 4               # the input grid is unchanged


def test_complete_max_edges_restores_a_dropped_top_row():
    grid = Grid(3, 2, 0.1, 1.0, 2.0, 0.0, (0,) * 6)
    # North wall hits only over the first two columns; the third new cell stays unknown.
    done = complete_max_edges(grid, _wall_hits([(1.05, 2.21), (1.15, 2.21)]), 0.01, 3)
    assert (done.width, done.height) == (3, 3)
    assert done.data[6:] == (100, 100, -1)


def test_complete_max_edges_leaves_a_complete_map_alone():
    grid = Grid(4, 3, 0.1, 0.0, 0.0, 0.0, (0,) * 12)
    # Hits inside the grid, too few hits past it, and hits two cells past it.
    hits = _wall_hits([(0.35, 0.15), (0.25, 0.05)])
    add_scan_hits(hits, [(0.45, 0.15)], 0.01)
    hits.update(_wall_hits([(0.55, 0.15)]))
    assert complete_max_edges(grid, hits, 0.01, 3) is grid


def test_complete_max_edges_ignores_hits_spilling_past_a_drawn_wall():
    # SLAM drew the east wall in its last column; most hits are there, a few spill past.
    data = tuple(100 if i == 3 else 0 for j in range(3) for i in range(4))
    grid = Grid(4, 3, 0.1, 0.0, 0.0, 0.0, data)
    hits = _wall_hits([(0.38, y) for y in (0.05, 0.15, 0.25)])
    hits.update(_wall_hits([(0.405, 0.15)], repeat=3))
    assert complete_max_edges(grid, hits, 0.01, 3) is grid


def test_frozen_map_is_latched_once_while_metadata_stays_live():
    node = SimpleNamespace(
        grid=Grid(2, 2, 0.1, 0.0, 0.0, 0.0, [0] * 4),
        frozen=True, frozen_published=False, meta_pub=Mock(), map_pub=Mock(),
        metadata=lambda: {}, get_clock=Mock())
    from builtin_interfaces.msg import Time
    node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
    MapManager.publish(node)
    MapManager.publish(node)
    assert node.map_pub.publish.call_count == 1
    assert node.meta_pub.publish.call_count == 2


def test_failed_snapshot_does_not_acknowledge_freeze():
    node = SimpleNamespace(
        state=SimpleNamespace(phase=CompetitionState.PHASE_HANDOVER), grid=object(),
        frozen=False, revision=1, stop_slam=Mock(),
        persist=Mock(side_effect=OSError('disk full')), publish=Mock(), metadata=lambda: {},
        robot_xy={})
    response = MapManager.freeze(node, Trigger.Request(), Trigger.Response())
    assert not response.success
    assert not node.frozen and node.revision == 1
    node.publish.assert_not_called()
    node.persist.side_effect = None
    response = MapManager.freeze(node, Trigger.Request(), Trigger.Response())
    assert response.success and node.revision == 2


def test_reloaded_frozen_map_reaches_late_subscriber(tmp_path):
    system = tmp_path / 'system'
    system.mkdir()
    (system / 'identity.json').write_text('{}')
    snapshot = {'run_id': tmp_path.name, 'map_id': 'map-12345678', 'revision': 2,
                'frozen': True, 'grid': {
                    'width': 2, 'height': 2, 'resolution': 0.1,
                    'origin_x': -1.0, 'origin_y': 2.0, 'origin_yaw': 0.0,
                    'data': [0, 100, -1, 0]}}
    (system / 'map_snapshot.json').write_text(json.dumps(snapshot))
    rclpy.init()
    manager = MapManager(parameter_overrides=[
        Parameter('run_directory', value=str(tmp_path)), Parameter('start_slam', value=False)])
    consumer = Node('late_map_consumer')
    received = []
    consumer.create_subscription(OccupancyGrid, '/map', received.append,
                                 qos_from_contract(load_runtime_contract(), 'snapshot'))
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    executor.add_node(consumer)
    try:
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert received
        assert list(received[0].data) == snapshot['grid']['data']
        assert received[0].info.origin.position.x == -1.0
        assert manager.frozen and manager.revision == 2 and manager.process is None
    finally:
        executor.shutdown()
        consumer.destroy_node()
        manager.destroy_node()
        rclpy.shutdown()
