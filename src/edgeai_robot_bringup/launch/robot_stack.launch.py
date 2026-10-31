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
Start the software stack of one robot in its namespace.

The stack is the safety_watchdog (ADR-0003), the robot_agent and, with
nav_mode:=amcl|slam, Nav2 from edgeai_robot_navigation.
Competition parameters are resolved per run_mode with edgeai_common and
handed to the nodes as ROS parameters; a competition launch fails while
required official values are still unset.
"""

import os

from ament_index_python.packages import get_package_share_directory
from edgeai_common.config import find_config_dir, load_runtime_contract, resolve_parameters
from edgeai_common.frames import validate_robot_id
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _as_bool(text):
    return text.strip().lower() in ('true', '1', 'yes')


def _launch_setup(context, *args, **kwargs):
    robot_id = validate_robot_id(LaunchConfiguration('robot_id').perform(context))
    run_mode = LaunchConfiguration('run_mode').perform(context)
    config_dir = str(find_config_dir(LaunchConfiguration('config_dir').perform(context) or None))
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    require_state = _as_bool(LaunchConfiguration('require_competition_state').perform(context))

    parameters = resolve_parameters(run_mode, config_dir)
    timeouts = load_runtime_contract(config_dir)['timeouts_s']

    watchdog = Node(
        package='edgeai_robot_bringup',
        executable='safety_watchdog',
        name='safety_watchdog',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'run_mode': run_mode,
            'config_dir': config_dir,
            'max_linear_mps': float(parameters.require('motion.max_linear_speed_mps')),
            'max_angular_radps': float(parameters.require('motion.max_angular_speed_radps')),
            'input_timeout_s': float(timeouts['velocity_command']),
            'edge_timeout_s': float(timeouts['edge_connection']),
            'require_competition_state': require_state,
            'use_sim_time': use_sim_time,
        }],
    )
    nav_mode = LaunchConfiguration('nav_mode').perform(context)
    if nav_mode not in ('none', 'amcl', 'slam'):
        raise RuntimeError(f'nav_mode must be none, amcl or slam, got {nav_mode!r}')
    agent = Node(
        package='edgeai_robot_bringup',
        executable='robot_agent',
        name='robot_agent',
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'run_mode': run_mode,
            'config_dir': config_dir,
            'require_competition_state': require_state,
            # Without Nav2 there is no AMCL either; the agent then reports localized=false.
            'localization_mode': 'slam' if nav_mode == 'slam' else 'amcl',
            'use_sim_time': use_sim_time,
        }],
    )
    actions = [GroupAction([PushRosNamespace(robot_id), watchdog, agent])]
    if nav_mode != 'none':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('edgeai_robot_navigation'), 'launch',
                'nav_robot.launch.py')),
            launch_arguments={
                'robot_id': robot_id,
                'mode': nav_mode,
                'run_mode': run_mode,
                'config_dir': config_dir,
                'use_sim_time': str(use_sim_time).lower(),
                'require_competition_state': str(require_state).lower(),
                'parked_start_pose_id':
                    LaunchConfiguration('parked_start_pose_id').perform(context),
                'parked_map_origin_pose_id':
                    LaunchConfiguration('parked_map_origin_pose_id').perform(context),
            }.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('run_mode', default_value='development',
                              description='development | rehearsal | competition'),
        DeclareLaunchArgument('config_dir', default_value='',
                              description='Repository config directory (default: search)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('require_competition_state', default_value='true',
                              description='false only in development: move without a '
                                          'competition phase manager'),
        DeclareLaunchArgument('nav_mode', default_value='none',
                              description='none | amcl | slam (Nav2 in edgeai_robot_navigation)'),
        DeclareLaunchArgument('parked_start_pose_id', default_value='',
                              description='nav_mode:=amcl only: wait parked at this start pose '
                                          '(visible to the other robots) until relocalized'),
        DeclareLaunchArgument('parked_map_origin_pose_id', default_value='',
                              description='Start pose at which the SLAM map is anchored '
                                          '(empty: map is the arena frame)'),
        OpaqueFunction(function=_launch_setup),
    ])
