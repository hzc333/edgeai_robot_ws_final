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

from dataclasses import asdict, replace
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from edgeai_common.config import find_config_dir, load_runtime_contract
from edgeai_common.qos import qos_from_contract
from edgeai_robot_interfaces.msg import CompetitionState, RobotState
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .frontier import Grid
from .ros_grid import from_message, to_message, yaw_of

# Lattice [m] on which the mapping robot's scan end points are counted (edge completion).
EDGE_LATTICE_M = 0.01


def clear_discs(grid, centers, radius):
    """Mark cells within radius of each (x, y) map point free (robots are not walls)."""
    data = list(grid.data)
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    reach = int(math.ceil(radius / grid.resolution))
    for x, y in centers:
        dx, dy = x - grid.origin_x, y - grid.origin_y
        cx = math.floor((c * dx + s * dy) / grid.resolution)
        cy = math.floor((-s * dx + c * dy) / grid.resolution)
        for j in range(cy - reach, cy + reach + 1):
            for i in range(cx - reach, cx + reach + 1):
                if (0 <= i < grid.width and 0 <= j < grid.height and math.hypot(
                        (i - cx) * grid.resolution, (j - cy) * grid.resolution) <= radius):
                    data[j * grid.width + i] = 0
    return replace(grid, data=tuple(data))


def center_karto_cells(grid):
    """
    Move the origin of a SLAM Toolbox grid back by half a cell along both grid axes.

    Karto rounds a point to the cell whose centre is origin + k * resolution, but SLAM
    Toolbox publishes that origin as the corner of cell 0 (the ROS convention), so its
    maps lie half a cell (2.5 cm at 0.05 m) towards +x/+y of the world. On the midterm
    maps this alone marked 2.8 % of the free cells next to walls as occupied.
    """
    half = grid.resolution / 2
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    return replace(grid, origin_x=grid.origin_x - half * (c - s),
                   origin_y=grid.origin_y - half * (s + c))


def add_scan_hits(hits, points, lattice):
    """Count scan end points (map x, y) per lattice cell of `lattice` metres."""
    for x, y in points:
        key = (math.floor(x / lattice), math.floor(y / lattice))
        hits[key] = hits.get(key, 0) + 1


def complete_max_edges(grid, hits, lattice, min_hits):
    """
    Add the column past +x and the row past +y that SLAM Toolbox can drop.

    Karto sizes its grid as round(extent / resolution) and rounds each point to the
    nearest cell, so the hits on the far (+x, +y) edge can round to the index one past
    the grid and are dropped: the east or north wall was missing from about half the
    midterm maps. Expects a grid whose origin is the corner of cell 0
    (center_karto_cells).
    A side is extended only when more hits fall just past it than in its last column
    (row): then the wall lies outside. Hits registered through the live TF also spill a
    cell past a wall SLAM did draw, and those leave the grid alone. In a new column/row,
    cells hit at least min_hits times become occupied, the rest unknown; the input grid
    is unchanged.
    """
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    cells = {}
    for (kx, ky), count in hits.items():
        dx = (kx + 0.5) * lattice - grid.origin_x
        dy = (ky + 0.5) * lattice - grid.origin_y
        cell = (math.floor((c * dx + s * dy) / grid.resolution),
                math.floor((-s * dx + c * dy) / grid.resolution))
        if 0 <= cell[0] <= grid.width and 0 <= cell[1] <= grid.height:
            cells[cell] = cells.get(cell, 0) + count

    def line(index, axis):
        return sum(n for cell, n in cells.items() if cell[axis] == index)

    extra_x = line(grid.width, 0) > line(grid.width - 1, 0)
    extra_y = line(grid.height, 1) > line(grid.height - 1, 1)
    walls = {(i, j) for (i, j), n in cells.items() if n >= min_hits and (
        (extra_x and i == grid.width) or (extra_y and j == grid.height))}
    if not walls:
        return grid
    extra_x = any(i == grid.width for i, _ in walls)
    extra_y = any(j == grid.height for _, j in walls)
    width, height = grid.width + extra_x, grid.height + extra_y
    data = [grid.data[j * grid.width + i] if i < grid.width and j < grid.height
            else 100 if (i, j) in walls else -1
            for j in range(height) for i in range(width)]
    return replace(grid, width=width, height=height, data=tuple(data))


class MapManager(Node):
    def __init__(self, **kwargs):
        super().__init__('map_manager', namespace='/edge', **kwargs)
        # odom_frame: SLAM's odometry frame. robot_1/arena_odom (P1 odom_origin_pose_id)
        # makes the SLAM map the arena frame from the start.
        # clear_robot_ids: robots that stood still in the mapping robot's view (robot_2
        # parked in the start area); their bodies are cleared from the frozen map so the
        # Task 2 planners do not see them as walls (P1 integration, midterm check).
        # edge_scan_topic: the mapping robot's scan; its end points put back the map edge
        # SLAM Toolbox can drop (complete_max_edges). Empty disables edge completion.
        # center_slam_cells: undo SLAM Toolbox's half-cell map offset (center_karto_cells).
        for name, value in [('run_directory', ''), ('config_dir', ''), ('slam_params', ''),
                            ('start_slam', True), ('odom_frame', 'robot_1/odom'),
                            ('clear_robot_ids', ''), ('clear_robot_radius_m', 0.25),
                            ('edge_scan_topic', ''), ('edge_min_hits', 3),
                            ('center_slam_cells', False)]:
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
        self.robot_xy = {}
        for robot in filter(None, self.get_parameter('clear_robot_ids').value.split(',')):
            self.create_subscription(
                RobotState, f'/{robot}/state',
                lambda m, r=robot: self.robot_xy.__setitem__(
                    r, (m.pose.pose.position.x, m.pose.pose.position.y)),
                qos_from_contract(self.contract, 'robot_state'))
        self.edge_hits = {}
        if self.get_parameter('edge_scan_topic').value:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(LaserScan, self.get_parameter('edge_scan_topic').value,
                                     self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '/edge/map_manager/freeze', self.freeze)
        # system_manager checks this heartbeat against the wall clock (3 s): a sim-time
        # timer would miss it whenever the simulation runs slower than about 0.33x.
        self.create_timer(1.0, self.publish_metadata,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
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
                 'odom_frame:=' + self.get_parameter('odom_frame').value,
                 'use_sim_time:=' + str(self.get_parameter('use_sim_time').value).lower()],
                stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)

    def on_scan(self, scan):
        """Count the mapping robot's scan end points in the map frame while mapping."""
        if self.frozen:
            return
        try:
            t = self.tf_buffer.lookup_transform('map', scan.header.frame_id,
                                                Time.from_msg(scan.header.stamp))
        except TransformException:
            return                      # before SLAM publishes map -> odom
        x, y = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        add_scan_hits(self.edge_hits, [
            (x + r * math.cos(yaw + scan.angle_min + k * scan.angle_increment),
             y + r * math.sin(yaw + scan.angle_min + k * scan.angle_increment))
            for k, r in enumerate(scan.ranges) if scan.range_min < r < scan.range_max],
            EDGE_LATTICE_M)

    def on_map(self, message):
        if self.frozen:
            return
        try:
            self.grid = from_message(message)
            if self.get_parameter('center_slam_cells').value:
                self.grid = center_karto_cells(self.grid)
            if self.edge_hits:
                self.grid = complete_max_edges(self.grid, self.edge_hits, EDGE_LATTICE_M,
                                               self.get_parameter('edge_min_hits').value)
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
                if self.robot_xy:
                    self.grid = clear_discs(self.grid, self.robot_xy.values(),
                                            self.get_parameter('clear_robot_radius_m').value)
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
