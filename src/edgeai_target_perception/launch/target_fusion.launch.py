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

"""Launch the centralized target fusion node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the edge-side fusion node launch description."""
    return LaunchDescription([
        DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2,robot_3,robot_4'),
        DeclareLaunchArgument('run_mode', default_value='development'),
        DeclareLaunchArgument('config_dir', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='edgeai_target_perception',
            executable='target_fusion',
            name='target_fusion',
            output='screen',
            parameters=[{
                'robot_ids': LaunchConfiguration('robot_ids'),
                'run_mode': LaunchConfiguration('run_mode'),
                'config_dir': LaunchConfiguration('config_dir'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
