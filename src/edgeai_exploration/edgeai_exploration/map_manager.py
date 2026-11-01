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

"""Own the authoritative map stream and SLAM subprocess lifetime.

Internal metadata uses std_msgs/String JSON, not a replacement public interface.
SLAM's map topic is remapped; its TF is stopped before frozen metadata is emitted.
"""

from dataclasses import asdict
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from edgeai_common.config import find_config_dir, load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .frontier import Grid
from .ros_grid import from_message, to_message


class MapManager(Node):
    def __init__(self, **kwargs):
        super().__init__('map_manager', namespace='/edge', **kwargs)
        for name, value in [('run_directory', ''), ('config_dir', ''), ('slam_params', ''),
                            ('start_slam', True)]:
            self.declare_parameter(name, value)
        self.directory = Path(self.get_parameter('run_directory').value).resolve()
        if not (self.directory / 'system/identity.json').is_file():
            raise ValueError('run_directory must contain system_manager identity')
        self.lease = (self.directory / 'system/map_owner.lock').open('a')
        fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.run_id = self.directory.name
        self.config = find_config_dir(self.get_parameter('config_dir').value)
        self.contract = load_runtime_contract(self.config)
        self.state = None
        self.grid = None
        self.map_received = None
        self.process = None
        self.log = None
        self.frozen = False
        self.frozen_published = False
        self.revision = 0
        self.map_id = 'map-' + uuid.uuid4().hex[:8]
        self.snapshot_path = self.directory / 'system/map_snapshot.json'
        if self.snapshot_path.exists():
            saved = json.loads(self.snapshot_path.read_text())
            if saved['run_id'] != self.run_id:
                raise ValueError('saved map belongs to another run')
            self.map_id, self.revision = saved['map_id'], saved['revision']
            self.frozen = saved['frozen']
            if saved['grid'] is not None:
                self.grid = Grid(**saved['grid'])
            # Never silently restart a previously running mapping session.
            if not self.frozen and self.grid is not None:
                raise ValueError('interrupted SLAM session requires explicit recovery/new run')
        else:
            self.persist()
        qos = qos_from_contract(self.contract, 'snapshot')
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.meta_pub = self.create_publisher(String, '/edge/map_metadata', qos)
        self.create_subscription(OccupancyGrid, '/edge/slam_map', self.on_map, qos)
        self.create_subscription(CompetitionState, '/competition/state', self.on_state,
                                 qos_from_contract(self.contract, 'control_state'))
        self.create_service(Trigger, '/edge/map_manager/freeze', self.freeze)
        self.create_timer(1.0, self.publish_metadata)
        self.publish()

    def persist(self):
        value = {'run_id': self.run_id, 'map_id': self.map_id, 'revision': self.revision,
                 'frozen': self.frozen, 'grid': asdict(self.grid) if self.grid else None}
        temporary = self.snapshot_path.with_suffix('.tmp')
        with temporary.open('w') as stream:
            json.dump(value, stream, allow_nan=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.snapshot_path)
        fd = os.open(self.snapshot_path.parent, os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)

    def on_state(self, message):
        if message.run_id != self.run_id:
            return
        self.state = message
        if (not self.frozen and self.process is None and message.phase in (0, 1)
                and self.get_parameter('start_slam').value):
            params = self.get_parameter('slam_params').value
            if not Path(params).is_file():
                self.get_logger().error('slam_params is missing')
                return
            self.log = (self.directory / 'system/slam.log').open('ab')
            self.process = subprocess.Popen(
                ['ros2', 'launch', 'edgeai_exploration', 'slam.launch.py',
                 'slam_params_file:=' + params,
                 'use_sim_time:=' + str(self.get_parameter('use_sim_time').value).lower()],
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def on_map(self, message):
        if self.frozen:
            return
        try:
            self.grid = from_message(message)
            self.map_received = time.monotonic()
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        # A durable initial snapshot makes a running session detectable after crash.
        if self.revision == 0:
            self.revision = 1
            self.persist()
        self.publish()

    def stop_slam(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=5)
        self.process = None
        if self.log:
            self.log.close()
            self.log = None

    def freeze(self, request, response):
        del request
        if not self.state or self.state.phase != CompetitionState.PHASE_HANDOVER:
            response.message = 'freeze requires HANDOVER'
            return response
        if self.grid is None:
            response.message = 'no valid map'
            return response
        try:
            if not self.frozen:
                self.stop_slam()
                self.frozen = True
                self.revision += 1
                try:
                    self.persist()
                except OSError:
                    self.frozen = False
                    self.revision -= 1
                    raise
            self.publish()
            response.success = True
            response.message = json.dumps(self.metadata())
        except (OSError, subprocess.SubprocessError) as error:
            response.message = str(error)
        return response

    def metadata(self):
        return {'run_id': self.run_id, 'map_id': self.map_id, 'revision': self.revision,
                'frozen': self.frozen, 'has_map': self.grid is not None,
                'slam_running': self.process is not None and self.process.poll() is None,
                'map_age_s': (time.monotonic() - self.map_received
                              if self.map_received is not None else None),
                'received_monotonic': time.monotonic()}

    def publish_metadata(self):
        self.meta_pub.publish(String(data=json.dumps(self.metadata())))

    def publish(self):
        self.meta_pub.publish(String(data=json.dumps(self.metadata())))
        if self.grid and not (self.frozen and self.frozen_published):
            self.map_pub.publish(to_message(self.grid, self.get_clock().now().to_msg()))
            self.frozen_published = self.frozen

    def destroy_node(self):
        self.stop_slam()
        self.lease.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
