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

"""ROS action client driving deterministic single-robot Frontier exploration."""

import math
import time
import uuid

from edgeai_common.config import find_config_dir, load_runtime_contract, resolve_parameters
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.action import ExecuteNavigationGoal
from edgeai_robot_interfaces.msg import CompetitionState, ExplorationStatus, RobotState
from edgeai_robot_interfaces.srv import SetCompetitionPhase
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from .frontier import ExplorationProgress, FrontierPlanner
from .ros_grid import from_message


class ExplorationCoordinator(Node):
    def __init__(self, **kwargs):
        super().__init__('exploration_coordinator', namespace='/edge', **kwargs)
        for name, value in [('robot_id', 'robot_1'), ('config_dir', ''),
                            ('run_mode', 'development'), ('clearance_m', 0.45),
                            ('goal_margin_m', 0.15),
                            ('minimum_frontier_cells', 3), ('approach_m', 0.8),
                            ('no_frontiers_s', 15.0), ('reference_area_m2', 0.0),
                            ('goal_timeout_s', 90.0), ('visited_radius_m', 0.25)]:
            self.declare_parameter(name, value)
        self.robot_id = self.get_parameter('robot_id').value
        if self.robot_id != 'robot_1':
            raise ValueError('ADR-0001 permits only robot_1 to explore the authoritative map')
        config = find_config_dir(self.get_parameter('config_dir').value)
        self.contract = load_runtime_contract(config)
        self.resolved = resolve_parameters(self.get_parameter('run_mode').value, config)
        self.planner = FrontierPlanner(
            self.get_parameter('clearance_m').value,
            self.get_parameter('minimum_frontier_cells').value,
            self.get_parameter('approach_m').value,
            goal_margin_m=self.get_parameter('goal_margin_m').value)
        self.progress = None
        self.state, self.robot, self.grid = None, None, None
        self.robot_received = 0.0
        self.map_received = 0.0
        self.goal_handle = None
        self.pending = False
        self.cancelling = False
        self.current = None
        self.visited = []
        self.reason = None
        self.end_future = None
        self.start_ns = None
        self.last_ns = None
        self.last_run = None
        self.count = 0
        control = qos_from_contract(self.contract, 'control_state')
        self.status_pub = self.create_publisher(
            ExplorationStatus, '/edge/exploration_status',
            qos_from_contract(self.contract, 'snapshot'))
        self.create_subscription(CompetitionState, '/competition/state', self.on_state, control)
        self.create_subscription(RobotState, f'/{self.robot_id}/state', self.on_robot,
                                 qos_from_contract(self.contract, 'robot_state'))
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 qos_from_contract(self.contract, 'snapshot'))
        self.action = ActionClient(self, ExecuteNavigationGoal,
                                   f'/{self.robot_id}/execute_navigation_goal')
        self.phase_client = self.create_client(SetCompetitionPhase, '/competition/set_phase')
        self.create_timer(1.0, self.tick)

    def on_state(self, message):
        self.state = message
        if message.phase != 1 or message.estop_active:
            self.cancel()

    def on_robot(self, message):
        if message.robot_id == self.robot_id:
            self.robot, self.robot_received = message, time.monotonic()

    def on_map(self, message):
        try:
            self.grid = from_message(message)
            self.map_received = time.monotonic()
        except ValueError as error:
            self.get_logger().error(str(error))

    def cancel(self):
        if self.goal_handle is not None and not self.cancelling:
            self.cancelling = True
            self.goal_handle.cancel_goal_async()

    def start_run(self):
        self.progress = ExplorationProgress(
            self.resolved.get('timing.task1_s', 600.0),
            self.resolved.get('exploration.no_progress_timeout_s', 20.0),
            self.get_parameter('no_frontiers_s').value,
            self.resolved.get('exploration.frontier_blacklist_radius_m', 0.3))
        self.visited = []
        self.reason = None
        self.start_ns = self.get_clock().now().nanoseconds
        self.last_ns = self.start_ns
        self.last_run = self.state.run_id

    def publish(self, status, detail):
        message = ExplorationStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        if self.state:
            message.run_id, message.map_id = self.state.run_id, self.state.map_id
            message.map_revision = self.state.map_revision
        message.state = status
        message.active_robot_ids = [self.robot_id] if status == 1 else []
        reference = self.get_parameter('reference_area_m2').value or None
        message.coverage_fraction = float(self.grid.metrics(reference)['coverage_fraction']) if (
            self.grid is not None) else float('nan')
        message.active_frontier_count = self.count
        elapsed = max(0, self.get_clock().now().nanoseconds - (self.start_ns or 0))
        message.elapsed.sec, message.elapsed.nanosec = divmod(elapsed, 1000000000)
        message.reason.detail = detail
        self.status_pub.publish(message)

    def finish(self):
        self.cancel()
        if self.pending or self.goal_handle is not None:
            return
        if self.end_future is not None:
            if not self.end_future.done():
                return
            self.end_future = None
        if not self.phase_client.service_is_ready():
            return
        request = SetCompetitionPhase.Request()
        request.run_id = self.state.run_id
        request.requested_phase, request.requester_type = 2, 2
        request.requester_node = '/edge/exploration_coordinator'
        request.reason = self.reason
        self.end_future = self.phase_client.call_async(request)

    def tick(self):
        if self.state is None or self.state.phase != 1 or self.state.estop_active:
            self.publish(2 if self.last_run else 0, 'waiting_for_task1')
            return
        if self.last_run != self.state.run_id:
            if self.pending or self.goal_handle:
                self.cancel()
                return
            self.start_run()
        now = self.get_clock().now().nanoseconds
        if now < self.last_ns:
            self.reason = 'clock_reset'
        self.last_ns = now
        if self.reason:
            self.publish(4 if self.reason == 'clock_reset' else 3, self.reason)
            self.finish()
            return
        elapsed = (now - self.start_ns) / 1e9
        if elapsed >= self.progress.time_limit:
            self.reason = 'time_limit'
            self.finish()
            return
        stale = self.contract['timeouts_s']['robot_state_stale']
        if (self.robot is None or self.robot.run_id != self.state.run_id
                or time.monotonic() - self.robot_received > stale
                or not self.robot.localized or not self.robot.navigation_available
                or self.grid is None or time.monotonic() - self.map_received > 10):
            self.cancel()
            self.publish(2, 'waiting_for_fresh_map_localization_and_navigation')
            return
        if self.pending or self.goal_handle:
            self.publish(1, 'navigating')
            return
        xy = self.robot.pose.pose.position
        candidates = self.planner.candidates(
            self.grid, (xy.x, xy.y), self.progress.blacklist() + self.visited)
        self.count = len(candidates)
        self.reason = self.progress.update(elapsed, self.grid.metrics()['known_area_m2'],
                                           self.count)
        if self.reason:
            self.finish()
            return
        self.publish(1, 'selecting_frontier' if candidates else 'no_reachable_frontiers')
        if not candidates or not self.action.server_is_ready():
            return
        self.current = candidates[0]
        goal = ExecuteNavigationGoal.Goal()
        goal.run_id, goal.map_id = self.state.run_id, self.state.map_id
        goal.map_revision = self.state.map_revision
        goal.request_id = 'explore-' + uuid.uuid4().hex
        goal.purpose = ExecuteNavigationGoal.Goal.PURPOSE_EXPLORATION
        goal.goal_pose.header.frame_id = 'map'
        # Map-fixed waypoints use latest TF throughout long Nav2 replanning.
        goal.goal_pose.header.stamp.sec = 0
        goal.goal_pose.header.stamp.nanosec = 0
        goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y = (
            self.current.x, self.current.y)
        goal.goal_pose.pose.orientation.z = math.sin(self.current.yaw / 2)
        goal.goal_pose.pose.orientation.w = math.cos(self.current.yaw / 2)
        goal.xy_tolerance_m, goal.yaw_tolerance_rad = 0.10, 0.0
        goal.timeout.sec = int(self.get_parameter('goal_timeout_s').value)
        self.pending = True
        self.action.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.progress.failed(self.current.x, self.current.y)
                return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.completed)
            if self.state.phase != 1 or self.reason:
                self.cancel()
        except Exception as error:
            self.get_logger().error('navigation acceptance failed: ' + str(error))
            self.progress.failed(self.current.x, self.current.y)

    def completed(self, future):
        try:
            result = future.result().result
            if result.success:
                self.visited.append((self.current.x, self.current.y,
                                     self.get_parameter('visited_radius_m').value))
            elif not self.cancelling:
                self.progress.failed(self.current.x, self.current.y)
        except Exception as error:
            self.get_logger().error('navigation result failed: ' + str(error))
            self.progress.failed(self.current.x, self.current.y)
        finally:
            self.goal_handle = None
            self.cancelling = False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExplorationCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cancel()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
