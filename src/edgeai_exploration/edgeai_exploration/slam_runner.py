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
SLAM Toolbox on the mapping robot's computer, started and stopped by map_manager.

During Task 1 SLAM publishes map -> <robot>/odom, which the mapping robot's Nav2 needs
in its control loop; on the edge node a late transform over the network aborts its
navigation (ADR-0004). This node runs on the robot and owns the SLAM process:

  ~/start   std_srvs/Trigger  start slam.launch.py (no-op while running)
  ~/stop    std_srvs/Trigger  end its process group: SLAM's map -> odom stops with it
  ~/status  std_msgs/String   {"running": bool}, every second (wall clock) and on change

map_manager (slam_runner:=/robot_1/slam_runner) keeps the map authority on the edge and
receives the map on /edge/slam_map as before.
"""

import json
import os
from pathlib import Path
import signal
import subprocess

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class SlamProcess:
    """SLAM Toolbox (slam.launch.py) in its own process group."""

    def __init__(self, slam_params, odom_frame, use_sim_time, log_path=None):
        self.slam_params, self.odom_frame = slam_params, odom_frame
        self.use_sim_time, self.log_path = use_sim_time, log_path
        self.process = None
        self.log = None

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self):
        """Start SLAM unless it runs; raise OSError when it cannot."""
        if self.running():
            return
        if not Path(self.slam_params).is_file():
            raise OSError('slam_params is missing')
        self.log = Path(self.log_path).open('ab') if self.log_path else None
        self.process = subprocess.Popen(
            ['ros2', 'launch', 'edgeai_exploration', 'slam.launch.py',
             'slam_params_file:=' + self.slam_params, 'odom_frame:=' + self.odom_frame,
             'use_sim_time:=' + str(self.use_sim_time).lower()],
            stdout=self.log, stderr=subprocess.STDOUT if self.log else None,
            start_new_session=True)

    def stop(self):
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


class SlamRunner(Node):
    """Serve start/stop of the SLAM process and report whether it runs."""

    def __init__(self, **kwargs):
        super().__init__('slam_runner', **kwargs)
        for name, value in [('slam_params', ''), ('odom_frame', 'robot_1/odom'),
                            ('log_file', '')]:
            self.declare_parameter(name, value)
        self.slam = SlamProcess(self.get_parameter('slam_params').value,
                                self.get_parameter('odom_frame').value,
                                self.get_parameter('use_sim_time').value,
                                self.get_parameter('log_file').value or None)
        self.status_pub = self.create_publisher(
            String, '~/status', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(1.0, self.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def on_start(self, request, response):
        del request
        try:
            self.slam.start()
            response.success, response.message = True, 'SLAM running'
            self.get_logger().info('started SLAM')
        except (OSError, subprocess.SubprocessError) as error:
            response.message = str(error)
            self.get_logger().error(f'cannot start SLAM: {error}')
        self.publish_status()
        return response

    def on_stop(self, request, response):
        del request
        try:
            self.slam.stop()
            response.success, response.message = True, 'SLAM stopped'
            self.get_logger().info('stopped SLAM')
        except (OSError, subprocess.SubprocessError) as error:
            response.message = str(error)
        self.publish_status()
        return response

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({'running': self.slam.running()})))

    def destroy_node(self):
        self.slam.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SlamRunner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
