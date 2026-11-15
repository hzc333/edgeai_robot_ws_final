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
Start two mock robot agents (robot_1, robot_2) for edge-side development.

Never include this file in a competition launch: mocks only replace the real
robot agent during development and in system tests.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _mock(robot_id, x, y):
    return Node(
        package='edgeai_system_tests',
        executable='mock_robot_agent',
        namespace=robot_id,
        name='robot_agent',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'initial_x': x,
            'initial_y': y,
            'config_dir': LaunchConfiguration('config_dir'),
            'publish_tf': LaunchConfiguration('publish_tf'),
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=os.environ.get('EDGEAI_CONFIG_DIR', ''),
                              description='Repository config directory (default: search)'),
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='Publish map -> robot_N/base_footprint'),
        _mock('robot_1', -1.5, -1.5),
        _mock('robot_2', -1.5, -0.8),
    ])
