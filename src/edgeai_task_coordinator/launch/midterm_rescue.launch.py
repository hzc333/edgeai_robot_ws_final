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

"""Start only the coordinator; robot stacks, map, localization and phase run elsewhere."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('edgeai_task_coordinator'))
                 / 'config' / 'midterm.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('config_dir', default_value=''),
        DeclareLaunchArgument('output_dir', default_value=str(Path.cwd() / 'runs' / 'rescue')),
        Node(package='edgeai_task_coordinator', executable='coordinator',
             output='screen', parameters=[{
                 'use_sim_time': True,
                 'config_file': LaunchConfiguration('config_file'),
                 'config_dir': LaunchConfiguration('config_dir'),
                 'output_dir': LaunchConfiguration('output_dir'),
             }]),
    ])
