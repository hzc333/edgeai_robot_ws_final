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

"""Tests for the peer_footprints node (P1 plan 3-01)."""

import math
import threading
import time

from edgeai_robot_navigation.peer_footprints import (
    grow,
    outline,
    PeerFootprints,
    transform_points,
)
from geometry_msgs.msg import TransformStamped
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster

FOOTPRINT = [[0.115, 0.175], [0.115, -0.175], [-0.315, -0.175], [-0.315, 0.175]]


def test_outline_closes_the_polygon_at_the_spacing():
    points = outline(FOOTPRINT, 0.05)
    assert len(points) == 2 * (7 + 9)          # 0.35 m and 0.43 m sides
    assert points[0] == pytest.approx((0.115, 0.175))
    gaps = [math.dist(points[i], points[(i + 1) % len(points)]) for i in range(len(points))]
    assert max(gaps) <= 0.05 + 1e-9


def test_grow_pads_every_side():
    grown = grow(FOOTPRINT, 0.08)
    assert grown == pytest.approx([(0.195, 0.255), (0.195, -0.255), (-0.395, -0.255),
                                   (-0.395, 0.255)])
    assert grow(FOOTPRINT, 0.0) == [tuple(v) for v in FOOTPRINT]


def test_transform_points():
    x, y = transform_points([(1.0, 0.0)], 1.0, 2.0, math.pi / 2)[0]
    assert (x, y) == pytest.approx((1.0, 3.0))


class World(Node):
    """TF of robot_1 (localized), robot_2 one metre ahead and a stale robot_3."""

    def __init__(self):
        super().__init__('fake_world')
        self.tf = TransformBroadcaster(self)
        self.robot_3_stamp = None
        self.create_timer(0.05, self._publish)

    def _tf(self, parent, child, x, stamp):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
        t.transform.translation.x = x
        t.transform.rotation.w = 1.0
        return t

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self.robot_3_stamp = self.robot_3_stamp or now      # published once, then stale
        self.tf.sendTransform([
            self._tf('map', 'robot_1/odom', 0.0, now),
            self._tf('robot_1/odom', 'robot_1/base_footprint', 0.0, now),
            self._tf('map', 'robot_2/base_footprint', 1.0, now),
            self._tf('map', 'robot_3/base_footprint', -1.0, self.robot_3_stamp),
        ])


def test_publishes_only_fresh_peers_in_own_base_frame():
    rclpy.init()
    node = PeerFootprints(parameter_overrides=[
        rclpy.parameter.Parameter('peer_timeout_s', value=0.5)])
    world = World()
    listener = Node('listener')
    clouds = []
    listener.create_subscription(PointCloud2, 'peer_footprints', clouds.append, 10)
    executor = MultiThreadedExecutor(num_threads=4)
    for n in (node, world, listener):
        executor.add_node(n)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        time.sleep(2.0)
        assert clouds, 'no cloud published'
        cloud = clouds[-1]
        assert cloud.header.frame_id == 'robot_1/base_footprint'
        points = list(point_cloud2.read_points(cloud, field_names=('x', 'y', 'z')))
        # robot_2 only, its footprint padded by the default 0.08 m.
        assert len(points) == len(outline(grow(FOOTPRINT, 0.08), 0.05))
        xs = [p[0] for p in points]
        assert min(xs) == pytest.approx(1.0 - 0.315 - 0.08, abs=1e-3)
        assert max(xs) == pytest.approx(1.0 + 0.115 + 0.08, abs=1e-3)
    finally:
        executor.shutdown()
        for n in (node, world, listener):
            n.destroy_node()
        rclpy.shutdown()
