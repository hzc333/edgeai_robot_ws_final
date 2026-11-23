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

"""Run Task 2A and Task 2B against an existing simulation's frozen Task 1 products."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _coordinator(context):
    run = Path(LaunchConfiguration('run_directory').perform(context)).resolve()
    return [Node(
        package='edgeai_task_coordinator', executable='coordinator', output='screen',
        parameters=[{
            'use_sim_time': True,
            'config_dir': LaunchConfiguration('config_dir'),
            'robot_ids': LaunchConfiguration('robot_ids'),
            'frozen_targets': str(run / 'task1/targets.json'),
            'parameters_file': str(run / 'system/parameters.yaml'),
            'output_dir': str(run / 'rescue/frozen'),
        }], additional_env={'GZ_PARTITION': 'edgeai-' + run.name})]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_directory', description='Directory managed by system_manager'),
        DeclareLaunchArgument('robot_ids', default_value='robot_1,robot_2'),
        DeclareLaunchArgument('config_dir', default_value=''),
        OpaqueFunction(function=_coordinator),
    ])
