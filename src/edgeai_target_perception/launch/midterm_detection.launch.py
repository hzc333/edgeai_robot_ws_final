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
Midterm Part 1 detection: one robot's target detector plus the evidence recorder.

Detections (marker ID, target type, camera-frame position, simulation time) and
annotated images go to output_dir/<run_id>/ (see detection_recorder.py).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('run_mode', default_value='development'),
        DeclareLaunchArgument('config_dir', default_value=''),
        DeclareLaunchArgument('use_depth', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value='runs/detections'),
        Node(
            package='edgeai_target_perception',
            executable='target_detector',
            name='target_detector',
            namespace=robot_id,
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'run_mode': LaunchConfiguration('run_mode'),
                'config_dir': LaunchConfiguration('config_dir'),
                'use_depth': LaunchConfiguration('use_depth'),
            }],
        ),
        Node(
            package='edgeai_target_perception',
            executable='detection_recorder',
            name='detection_recorder',
            output='screen',
            parameters=[{
                'robot_ids': robot_id,
                'output_dir': LaunchConfiguration('output_dir'),
            }],
        ),
    ])
