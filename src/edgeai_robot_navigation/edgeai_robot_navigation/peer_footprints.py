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
Other robots' footprints as obstacles for one robot (P1 plan 3-01).

The 2D LiDAR scans at 0.155 m, above the chassis of the other robots, and sees
only their 7 cm LiDAR housing: robots crossing head-on drove into each other.
This node looks up every other robot in the shared /tf (ADR-0002), samples its
footprint outline and publishes the points, in this robot's base_footprint frame,
on /<robot_id>/peer_footprints. Both costmaps and the collision monitor use the
cloud as an extra observation source.

Each outline is grown by padding_m: a peer's pose is its own AMCL estimate,
2-6 cm off in simulation, so with the collision monitor's 5 cm stop margin alone
two robots squeezing past each other came within 4 mm (0.15 m/s) or touched
(0.30 m/s) in the midterm arena.

A peer is included only while map -> robot_M/base_footprint is fresher than
peer_timeout_s, i.e. while that robot is localized and running. The cloud is
published at a fixed rate even when empty: the collision monitor stops the robot
when a source times out.
"""

import math

from edgeai_common.frames import frame, validate_robot_id
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

POINT_HEIGHT_M = 0.10      # inside every consumer's height window


def grow(footprint, padding):
    """Grow an axis-aligned rectangular footprint outward by padding on every side."""
    if padding <= 0.0:
        return [tuple(v) for v in footprint]
    cx = sum(x for x, _ in footprint) / len(footprint)
    cy = sum(y for _, y in footprint) / len(footprint)
    return [(x + math.copysign(padding, x - cx), y + math.copysign(padding, y - cy))
            for x, y in footprint]


def outline(footprint, spacing):
    """Points every `spacing` metres along the closed footprint polygon."""
    points = []
    for i, (x0, y0) in enumerate(footprint):
        x1, y1 = footprint[(i + 1) % len(footprint)]
        steps = max(1, math.ceil(math.hypot(x1 - x0, y1 - y0) / spacing))
        points.extend((x0 + (x1 - x0) * k / steps, y0 + (y1 - y0) * k / steps)
                      for k in range(steps))
    return points


def transform_points(points, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in points]


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PeerFootprints(Node):

    def __init__(self, **kwargs):
        super().__init__('peer_footprints', **kwargs)
        p = self.declare_parameter
        self.robot_id = validate_robot_id(p('robot_id', 'robot_1').value)
        peers = p('peers', ['robot_1', 'robot_2', 'robot_3', 'robot_4']).value
        self.peers = [validate_robot_id(r) for r in peers if r != self.robot_id]
        footprint = yaml.safe_load(p('footprint', '[[0.115, 0.175], [0.115, -0.175], '
                                                  '[-0.315, -0.175], [-0.315, 0.175]]').value)
        self.padding = float(p('padding_m', 0.08).value)
        self.outline = outline(grow(footprint, self.padding),
                               float(p('point_spacing_m', 0.05).value))
        self.peer_timeout = Duration(seconds=float(p('peer_timeout_s', 1.0).value))
        self.base = frame(self.robot_id, 'base_footprint')
        self.odom = frame(self.robot_id, 'odom')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PointCloud2, 'peer_footprints', 10)
        self.create_timer(1.0 / float(p('rate_hz', 10.0).value), self._publish)
        self.get_logger().info(f'{self.robot_id} peer footprints for {self.peers}')

    def _peer_points(self, peer, now):
        try:
            tf = self.tf_buffer.lookup_transform(self.base, frame(peer, 'base_footprint'),
                                                 Time())
        except TransformException:
            return []            # the peer is not localized, or not running
        if now - Time.from_msg(tf.header.stamp) > self.peer_timeout:
            return []
        t = tf.transform
        return transform_points(self.outline, t.translation.x, t.translation.y,
                                yaw_from_quaternion(t.rotation))

    def _publish(self):
        now = self.get_clock().now()
        header = Header(frame_id=self.base)
        # Stamp with this robot's latest odometry so that consumers can always
        # transform the cloud (a stamp ahead of the TF would be dropped).
        try:
            header.stamp = self.tf_buffer.lookup_transform(self.odom, self.base,
                                                           Time()).header.stamp
        except TransformException:
            return
        points = [(x, y, POINT_HEIGHT_M) for peer in self.peers
                  for x, y in self._peer_points(peer, now)]
        self.pub.publish(point_cloud2.create_cloud_xyz32(header, points))


def main(args=None):
    rclpy.init(args=args)
    node = PeerFootprints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
