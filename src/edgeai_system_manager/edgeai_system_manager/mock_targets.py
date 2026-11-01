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

"""Explicit development-only empty P3 registry for P2 integration tests.

This fixture proves control/serialization paths only. It is not perception and
must never be enabled with a real target_fusion publisher or outside development.
"""

import json

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState, TargetArray
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EmptyTargetFixture(Node):
    def __init__(self):
        super().__init__('target_fusion', namespace='/edge')
        self.declare_parameter('run_mode', '')
        self.declare_parameter('config_dir', '')
        if self.get_parameter('run_mode').value != 'development':
            raise ValueError('empty target fixture is restricted to development')
        runtime = load_runtime_contract(self.get_parameter('config_dir').value)
        self.state = None
        self.meta = None
        snapshot = qos_from_contract(runtime, 'snapshot')
        self.publisher = self.create_publisher(TargetArray, '/edge/targets', snapshot)
        self.create_subscription(CompetitionState, '/competition/state', self.on_state,
                                 qos_from_contract(runtime, 'control_state'))
        self.create_subscription(String, '/edge/map_metadata', self.on_meta, snapshot)
        self.create_timer(0.2, self.publish)

    def on_state(self, message):
        self.state = message

    def on_meta(self, message):
        self.meta = json.loads(message.data)

    def publish(self):
        if self.state is None or self.meta is None:
            return
        if len(self.get_publishers_info_by_topic('/edge/targets')) > 1:
            raise RuntimeError('empty target fixture conflicts with another target authority')
        message = TargetArray()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.run_id = self.state.run_id
        message.map_id, message.map_revision = self.meta['map_id'], self.meta['revision']
        message.task1_frozen = self.meta['frozen']
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EmptyTargetFixture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
