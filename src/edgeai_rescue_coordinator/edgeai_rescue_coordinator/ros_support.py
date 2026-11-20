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

"""ROS conversion and explicit frozen Task 1 context for local Task 2B."""

import json
from pathlib import Path

from edgeai_common.config import load_runtime_contract
from edgeai_common.qos import qos_from_contract
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields


def to_dict(message):
    """Detach a ROS message into JSON-compatible field dictionaries."""
    return json.loads(json.dumps(message_to_ordereddict(message)))


def message(cls, fields):
    """Build a typed message with the repository's existing fields."""
    result = cls()
    set_message_fields(result, fields)
    return result


class ContextNode(Node):
    """Require an explicit frozen registry; never invent a live run identity."""

    def __init__(self, name):
        super().__init__(name, namespace='/edge')
        self.contract = load_runtime_contract(
            self.declare_parameter('config_dir', '').value or None)
        path = self.declare_parameter('frozen_targets', '').value
        if not path:
            raise ValueError('frozen_targets parameter is required')
        self.targets = json.loads(Path(path).read_text())
        self.run_context = {'run_id': self.targets['run_id'],
                            'map_id': self.targets['map']['map_id'],
                            'map_revision': self.targets['map']['revision'],
                            'target_revision': self.targets['revision']}

    def qos(self, kind):
        return qos_from_contract(self.contract, kind)

    def topic(self, key):
        return self.contract['topics'][key]

    def service(self, key):
        return self.contract['services'][key]
