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

"""Run the production coordinator as a separate, killable process for rehearsals."""

import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
from edgeai_common.qos import qos_from_contract
from edgeai_task_coordinator.model import load_configuration
from nav_msgs.msg import OccupancyGrid
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String


class EngineView:
    """Read published state only; the validation driver never executes coordinator logic."""

    def __init__(self, data, config):
        self.data = data
        self.status = data['status']
        self.target = next((t for t in config.targets if t.target_id == data.get('target_id')),
                           None)
        self.assignment = tuple(data.get('assignment', []))
        self.slots = {r: {'state': 'active' if data.get('task_state') == 'DWELLING' else 'queued'}
                      for r in self.assignment}

    def snapshot(self):
        return self.data


class RemoteCoordinator:
    """Observe real ROS state, retain logs, and SIGKILL/restart only our own process."""

    def __init__(self, control, contract, config_dir, output):
        self.config = load_configuration(Path(get_package_share_directory(
            'edgeai_task_coordinator')) / 'config' / 'midterm.yaml')
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.engine = None
        self.grid = None
        self.journal = None
        self.last_error = ''
        self.data = {}
        self.blocked_instance = None
        self.output = output
        self.log = (output / 'coordinator.log').open('w')
        self.command = ['ros2', 'run', 'edgeai_task_coordinator', 'coordinator', '--ros-args',
                        '-p', 'use_sim_time:=true', '-p', f'config_dir:={config_dir}',
                        '-p', f'output_dir:={output / "evidence"}']
        control.create_subscription(String, '/edge/midterm_rescue/status', self._state,
                                    qos_from_contract(contract, 'snapshot'))
        control.create_subscription(OccupancyGrid, '/map', self._map,
                                    qos_from_contract(contract, 'snapshot'))
        self.process = None
        self.start()

    def _map(self, msg):
        self.grid = msg

    def _state(self, msg):
        data = json.loads(msg.data)
        if data.get('coordinator_instance') == self.blocked_instance:
            return
        self.data = data
        self.last_error = data.get('error', '')
        if data.get('identity'):
            self.engine = EngineView(data, self.config)
            key = hashlib.sha256(data['identity']['run_id'].encode()).hexdigest()[:24]
            self.journal = SimpleNamespace(directory=self.output / 'evidence' / key)

    def start(self):
        self.process = subprocess.Popen(self.command, stdout=self.log, stderr=subprocess.STDOUT,
                                        start_new_session=True)

    def crash_restart(self):
        old_pid = self.process.pid
        self.blocked_instance = self.data.get('coordinator_instance')
        os.killpg(old_pid, signal.SIGKILL)
        self.process.wait(timeout=5)
        self.engine = None
        self.data = {}
        self.start()
        return {'old_process_group': old_pid, 'new_process_group': self.process.pid}

    def close(self):
        if self.process and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()
