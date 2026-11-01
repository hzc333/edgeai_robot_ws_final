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
Start the Gazebo world and one or more simulated robots (ADR-0002).

Usage: ros2 launch edgeai_robot_bringup sim_multi_robot.launch.py robots:=robot_1,robot_2

robots_file picks the spawn poses (config/sim/robots.yaml by default);
nav_modes and parked override nav_mode and parking per robot, e.g.
nav_modes:=robot_1:slam,robot_2:amcl parked:=robot_2:midterm_robot_2.
"""

import os

from ament_index_python.packages import get_package_share_directory
from edgeai_common.config import find_config_dir, load_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    return text.strip().lower() in ('true', '1', 'yes')


def _per_robot(text, name):
    """Parse 'robot_1:value,robot_2:value' into a dict."""
    result = {}
    for item in (i.strip() for i in text.split(',') if i.strip()):
        robot_id, sep, value = item.partition(':')
        if not sep or not value:
            raise RuntimeError(f'{name} entries must be robot_N:value, got {item!r}')
        result[robot_id.strip()] = value.strip()
    return result


def _launch_setup(context, *args, **kwargs):
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',')
              if r.strip()]
    if not robots or len(set(robots)) != len(robots):
        raise RuntimeError(f'robots must be a non-empty list without duplicates: {robots}')
    config_dir = find_config_dir(LaunchConfiguration('config_dir').perform(context) or None)
    robots_file = LaunchConfiguration('robots_file').perform(context)
    robots_path = config_dir / 'sim' / robots_file    # an absolute robots_file wins
    spawn_poses = load_yaml(robots_path)['robots']
    nav_modes = _per_robot(LaunchConfiguration('nav_modes').perform(context), 'nav_modes')
    parked = _per_robot(LaunchConfiguration('parked').perform(context), 'parked')
    unknown = (set(nav_modes) | set(parked)) - set(robots)
    if unknown:
        raise RuntimeError(f'nav_modes/parked name robots that are not spawned: {unknown}')

    description = get_package_share_directory('edgeai_robot_description')
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'sim_world.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world').perform(context),
            'headless': LaunchConfiguration('headless').perform(context),
        }.items(),
    )]
    for robot_id in robots:
        if robot_id not in spawn_poses:
            raise RuntimeError(f'{robot_id} has no spawn pose in {robots_path}')
        pose = spawn_poses[robot_id]
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, 'launch', 'sim_robot.launch.py')),
            launch_arguments={
                'robot_id': robot_id,
                'x': str(pose['x']),
                'y': str(pose['y']),
                'yaw': str(pose['yaw']),
            }.items(),
        ))
        if _as_bool(LaunchConfiguration('stack').perform(context)):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('edgeai_robot_bringup'), 'launch',
                    'robot_stack.launch.py')),
                launch_arguments={
                    'robot_id': robot_id,
                    'run_mode': LaunchConfiguration('run_mode').perform(context),
                    'config_dir': str(config_dir),
                    'use_sim_time': 'true',
                    'require_competition_state':
                        LaunchConfiguration('require_competition_state').perform(context),
                    'nav_mode': nav_modes.get(
                        robot_id, LaunchConfiguration('nav_mode').perform(context)),
                    'parked_start_pose_id': parked.get(robot_id, ''),
                    'parked_map_origin_pose_id':
                        LaunchConfiguration('parked_map_origin').perform(context),
                    # Explicit: include arguments are global and would leak between robots.
                    'odom_origin_pose_id': '',
                }.items(),
            ))

    if _as_bool(LaunchConfiguration('rviz').perform(context)):
        rviz_file = 'realistic_robot.rviz' if len(robots) == 1 else 'multi_robot.rviz'
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(description, 'rviz', rviz_file)],
            parameters=[{'use_sim_time': True}],
        ))
    return actions


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('edgeai_robot_description'), 'worlds', 'empty.sdf')
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='robot_1,robot_2',
                              description='Comma-separated robot IDs to spawn'),
        DeclareLaunchArgument('config_dir', default_value=os.environ.get('EDGEAI_CONFIG_DIR', ''),
                              description='Repository config directory (default: search)'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('robots_file', default_value='robots.yaml',
                              description='Spawn poses: absolute path or file in config/sim'),
        DeclareLaunchArgument('nav_modes', default_value='',
                              description='Per-robot nav_mode override, robot_N:mode,...'),
        DeclareLaunchArgument('parked', default_value='',
                              description='Per-robot parking start pose, robot_N:pose_id,...'),
        DeclareLaunchArgument('parked_map_origin', default_value='',
                              description='Start pose at which the SLAM map is anchored '
                                          '(empty: map is the arena frame)'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('stack', default_value='true',
                              description='Start each robot stack (robot_stack.launch.py)'),
        DeclareLaunchArgument('run_mode', default_value='development'),
        DeclareLaunchArgument('nav_mode', default_value='none',
                              description='none | amcl | slam for every robot stack'),
        DeclareLaunchArgument('require_competition_state', default_value='true',
                              description='false only in development (manual driving '
                                          'without a phase manager)'),
        OpaqueFunction(function=_launch_setup),
    ])
