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

"""Conversion between ROS map messages and validated planar grids."""

import math

from nav_msgs.msg import OccupancyGrid

from .frontier import Grid


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def from_message(message):
    if message.header.frame_id != 'map':
        raise ValueError('occupancy grid must be in map frame')
    origin = message.info.origin
    q = origin.orientation
    if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 1e-4:
        raise ValueError('map origin quaternion is not normalized')
    if abs(origin.orientation.x) > 1e-6 or abs(origin.orientation.y) > 1e-6:
        raise ValueError('non-planar map origin')
    return Grid(message.info.width, message.info.height, message.info.resolution,
                origin.position.x, origin.position.y, yaw_of(origin.orientation),
                tuple(message.data))


def to_message(grid, stamp):
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.header.stamp = stamp
    message.info.width, message.info.height = grid.width, grid.height
    message.info.resolution = grid.resolution
    message.info.origin.position.x, message.info.origin.position.y = grid.origin_x, grid.origin_y
    message.info.origin.orientation.z = math.sin(grid.origin_yaw / 2)
    message.info.origin.orientation.w = math.cos(grid.origin_yaw / 2)
    message.data = list(grid.data)
    return message
