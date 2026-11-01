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

"""P2 Task 1 entry point with stable run identity and optional simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from edgeai_common.config import find_config_dir
from edgeai_system_manager.bootstrap import initialize_artifacts
from edgeai_system_manager.storage import atomic_bytes, new_run_id, RunStore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import EmitEvent, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    config = find_config_dir(value('config_dir'))
    runs = Path(value('runs_root')).expanduser().resolve()
    runs.mkdir(parents=True, exist_ok=True)
    active = runs / 'active_run.txt'
    run_id = value('run_id')
    if not run_id:
        run_id = active.read_text().strip() if active.exists() else new_run_id()
    if run_id == 'new':
        run_id = new_run_id()
    mode = value('run_mode')
    robots = value('robots').split(',')
    if not robots or robots[0] != 'robot_1' or len(set(robots)) != len(robots):
        raise ValueError('robots must start with robot_1 and contain no duplicates')
    store = RunStore(runs, run_id, {'run_mode': mode, 'robots': robots})
    try:
        initialize_artifacts(store, config, mode, robots)
        frozen_map = (store.read('system/map_snapshot.json') or {}).get('frozen', False)
        atomic_bytes(active, (run_id + '\n').encode())
    finally:
        store.close()
    sim = value('simulation').lower() == 'true'
    shared = {'use_sim_time': sim, 'config_dir': str(config), 'run_mode': mode}
    exploration = Path(get_package_share_directory('edgeai_exploration'))
    actions = [
        Node(package='edgeai_system_manager', executable='system_manager', output='screen',
             parameters=[shared, {'run_id': run_id, 'runs_root': str(runs), 'robot_ids': robots}]),
        Node(package='edgeai_system_manager', executable='event_logger', output='screen',
             parameters=[{'use_sim_time': sim, 'config_dir': str(config),
                          'run_directory': str(runs / run_id),
                          'record_bag': value('record_bag').lower() == 'true'}]),
        Node(package='edgeai_exploration', executable='map_manager', output='screen',
             parameters=[{'use_sim_time': sim, 'config_dir': str(config),
                          'run_directory': str(runs / run_id),
                          'slam_params': str(exploration / 'config/slam.yaml')}]),
        Node(package='edgeai_exploration', executable='exploration_coordinator', output='screen',
             parameters=[shared]),
    ]
    if value('mock_targets').lower() == 'true':
        if mode != 'development':
            raise ValueError('mock_targets requires development mode')
        actions.append(Node(package='edgeai_system_manager', executable='mock_empty_targets',
                            output='screen', parameters=[shared]))
    if sim:
        actions.insert(0, SetEnvironmentVariable('GZ_PARTITION', 'p2-' + run_id))
        bringup = Path(get_package_share_directory('edgeai_robot_bringup'))
        actions.insert(1, IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(bringup / 'launch/sim_multi_robot.launch.py')),
            launch_arguments={'robots': ','.join(robots), 'stack': 'false',
                              'headless': value('headless'), 'rviz': value('rviz'),
                              'config_dir': str(config)}.items()))
        # Start only after observed map and TF readiness, independent of machine speed.
        stacks = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/robot_stack.launch.py')),
            launch_arguments={'robot_id': robot, 'run_mode': mode, 'use_sim_time': 'true',
                              'config_dir': str(config), 'require_competition_state': 'true',
                              'nav_mode': ('slam' if robot == 'robot_1' and not frozen_map
                                           else 'amcl')}.items())
                  for robot in robots]
        gate = Node(package='edgeai_exploration', executable='wait_mapping', output='screen',
                    parameters=[{'use_sim_time': True, 'require_tf': not frozen_map}])

        def mapping_ready(event, context):
            del context
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason='map/TF startup gate failed'))]
            return stacks

        actions += [RegisterEventHandler(OnProcessExit(target_action=gate,
                                                       on_exit=mapping_ready)), gate]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_dir', default_value=''),
        DeclareLaunchArgument('runs_root', default_value='runs'),
        DeclareLaunchArgument('run_id', default_value=''),
        DeclareLaunchArgument('run_mode', default_value='development'),
        DeclareLaunchArgument('robots', default_value='robot_1'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('mock_targets', default_value='false'),
        OpaqueFunction(function=setup),
    ])
