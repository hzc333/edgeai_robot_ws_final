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
Serve a saved map on the global /map for development (not a competition launch).

In competition runs /map comes from map_manager. This launch only lets the robot stack
test localization and navigation before that exists. The default map's frame
equals the Gazebo world frame, so Gazebo ground truth is directly in map
coordinates.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = os.path.join(get_package_share_directory('edgeai_robot_description'),
                               'maps', 'edgeai_arena_world.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map'),
                          'use_sim_time': use_sim_time, 'topic_name': 'map',
                          'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['map_server']}]),
    ])
