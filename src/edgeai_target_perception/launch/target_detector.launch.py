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

"""Launch the single-robot target detector under its robot namespace."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('run_mode', default_value='development'),
        DeclareLaunchArgument('config_dir', default_value=''),
        DeclareLaunchArgument('use_depth', default_value='false'),
        Node(
            package='edgeai_target_perception',
            executable='target_detector',
            name='target_detector',
            namespace=LaunchConfiguration('robot_id'),
            output='screen',
            parameters=[{
                'robot_id': LaunchConfiguration('robot_id'),
                'run_mode': LaunchConfiguration('run_mode'),
                'config_dir': LaunchConfiguration('config_dir'),
                'use_depth': LaunchConfiguration('use_depth'),
            }],
        ),
    ])
