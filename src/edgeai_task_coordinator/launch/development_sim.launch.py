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

"""Coordinator-only integration fixture: real robots/Nav2, no production phase owner."""

import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from edgeai_task_coordinator.physics import plugin_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _setup(context):
    share = Path(get_package_share_directory('edgeai_task_coordinator'))
    description = Path(get_package_share_directory('edgeai_robot_description'))
    bringup = Path(get_package_share_directory('edgeai_robot_bringup'))
    navigation = Path(get_package_share_directory('edgeai_robot_navigation'))
    config = LaunchConfiguration('config_dir').perform(context)

    def include(path, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                        launch_arguments=arguments.items())

    world = ET.parse(share / 'test_assets' / 'midterm.sdf')
    world.find(".//plugin[@name='edgeai::Contacts']").set('filename', plugin_path())
    with tempfile.NamedTemporaryFile(suffix='.sdf', prefix='rescue-world-', delete=False) as stream:
        world.write(stream, encoding='utf-8', xml_declaration=True)
        world_path = stream.name

    actions = [
        include(description / 'launch' / 'sim_world.launch.py', {
            'world': world_path,
            'headless': LaunchConfiguration('headless').perform(context)}),
        include(navigation / 'launch' / 'dev_map_server.launch.py', {
            'map': str(share / 'test_assets' / 'midterm.yaml'), 'use_sim_time': 'true'}),
    ]
    for robot_id in ('robot_1', 'robot_2'):
        start = LaunchConfiguration(robot_id + '_start').perform(context).split(',')
        if len(start) != 3:
            raise ValueError(robot_id + '_start must be x,y,yaw in metres/radians')
        x, y, yaw = (str(float(v)) for v in start)
        actions += [
            include(description / 'launch' / 'sim_robot.launch.py', {
                'robot_id': robot_id, 'x': x, 'y': y, 'yaw': yaw}),
            include(bringup / 'launch' / 'robot_stack.launch.py', {
                'robot_id': robot_id, 'run_mode': 'development', 'config_dir': config,
                'use_sim_time': 'true', 'require_competition_state': 'true', 'nav_mode': 'amcl'}),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=os.environ.get('EDGEAI_CONFIG_DIR', '')),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('robot_1_start', default_value='0.55,0.5,0.0'),
        DeclareLaunchArgument('robot_2_start', default_value='1.20,0.5,0.0'),
        OpaqueFunction(function=_setup),
    ])
