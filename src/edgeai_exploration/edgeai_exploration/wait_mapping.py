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

"""Gate Nav2 activation on an actual map and a connected TF tree."""

import time

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from .ros_grid import from_message


def main(args=None):
    rclpy.init(args=args)
    node = Node('wait_mapping', namespace='/edge')
    node.declare_parameter('timeout_s', 90.0)
    node.declare_parameter('require_tf', True)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    received = []

    def on_map(message):
        try:
            from_message(message)
            received[:] = [True]
        except ValueError as error:
            node.get_logger().error(str(error))

    node.create_subscription(OccupancyGrid, '/map', on_map,
                             qos_from_contract(load_runtime_contract(), 'snapshot'))
    deadline = time.monotonic() + node.get_parameter('timeout_s').value
    ready = False
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            tf_ready = buffer.can_transform('map', 'robot_1/base_footprint', Time())
            if received and (tf_ready or not node.get_parameter('require_tf').value):
                ready = True
                break
        if not ready:
            node.get_logger().error('map/TF startup timeout; Nav2 was not started')
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ready else 1
