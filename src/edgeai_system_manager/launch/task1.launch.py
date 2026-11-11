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
from edgeai_common.config import find_config_dir, resolve_parameters
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
    # scenario:=midterm (P1 integration, midterm check 2026-11-12): P4's arena, robot_2
    # parked in the start area, the SLAM map in the arena frame, P3 detection and the
    # P4 rescue coordinator; exploration ends at the marker viewpoints and back home.
    midterm = value('scenario') == 'midterm'
    if midterm:
        robots = ['robot_1', 'robot_2']
        if value('mock_targets').lower() == 'true':
            raise ValueError('scenario:=midterm uses P3 target fusion, not mock_targets')
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
    manager = {'run_id': run_id, 'runs_root': str(runs), 'robot_ids': robots}
    # SLAM Toolbox maps lie half a cell off and can drop their far edge; map_manager
    # corrects both (center_karto_cells, complete_max_edges with robot_1's scans; robot_1
    # is the only mapping robot, ADR-0001).
    mapping = {'use_sim_time': sim, 'config_dir': str(config),
               'run_directory': str(runs / run_id),
               'slam_params': str(exploration / 'config/slam.yaml'),
               'center_slam_cells': True, 'edge_scan_topic': '/robot_1/scan'}
    explorer = {}
    if midterm:
        manager['start_pose_ids'] = 'robot_2:midterm_robot_2'
        mapping['odom_frame'] = 'robot_1/arena_odom'
        mapping['clear_robot_ids'] = 'robot_2'
        explorer = {'viewpoints_file': str(config / 'sim/midterm_marker_viewpoints.yaml'),
                    'return_pose_id': 'midterm_robot_1'}
    actions = [
        Node(package='edgeai_system_manager', executable='system_manager', output='screen',
             parameters=[shared, manager]),
        Node(package='edgeai_system_manager', executable='event_logger', output='screen',
             parameters=[{'use_sim_time': sim, 'config_dir': str(config),
                          'run_directory': str(runs / run_id),
                          'record_bag': value('record_bag').lower() == 'true'}]),
        Node(package='edgeai_exploration', executable='map_manager', output='screen',
             parameters=[mapping]),
        Node(package='edgeai_exploration', executable='exploration_coordinator', output='screen',
             parameters=[shared, explorer]),
    ]
    if midterm:
        perception = {'use_sim_time': sim, 'run_mode': mode, 'config_dir': str(config)}
        actions += [Node(package='edgeai_target_perception', executable='target_detector',
                         name='target_detector', namespace=robot, output='screen',
                         parameters=[perception, {'robot_id': robot, 'use_depth': False}])
                    for robot in robots]
        actions += [
            Node(package='edgeai_target_perception', executable='target_fusion',
                 name='target_fusion', output='screen',
                 parameters=[perception, {'robot_ids': ','.join(robots)}]),
            Node(package='edgeai_target_perception', executable='detection_recorder',
                 name='detection_recorder', output='screen',
                 parameters=[{'use_sim_time': sim, 'robot_ids': ','.join(robots),
                              'output_dir': str(runs / run_id / 'detections')}]),
            # P4 waits for TASK2A, then rescues S1 and C1 and requests FROZEN.
            Node(package='edgeai_task_coordinator', executable='coordinator',
                 name='coordinator', output='screen',
                 parameters=[{'use_sim_time': sim, 'config_dir': str(config),
                              'output_dir': str(runs / run_id / 'p4')}]),
        ]
    if value('mock_targets').lower() == 'true':
        if mode != 'development':
            raise ValueError('mock_targets requires development mode')
        actions.append(Node(package='edgeai_system_manager', executable='mock_empty_targets',
                            output='screen', parameters=[shared]))
    if sim:
        actions.insert(0, SetEnvironmentVariable('GZ_PARTITION', 'p2-' + run_id))
        bringup = Path(get_package_share_directory('edgeai_robot_bringup'))
        world = (('launch/midterm_sim.launch.py', {'part': '1', 'run_mode': mode})
                 if midterm else ('launch/sim_multi_robot.launch.py',
                                  {'robots': ','.join(robots)}))
        actions.insert(1, IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(bringup / world[0])),
            launch_arguments={**world[1], 'stack': 'false',
                              'headless': value('headless'), 'rviz': value('rviz'),
                              'config_dir': str(config)}.items()))
        # midterm: map is the arena frame (robot_1/arena_odom); robot_2 stays visible
        # at its start pose until relocalized for TASK2A.
        # Every key for every robot: launch arguments of an include are global, so an
        # omitted one would inherit the previous robot's value (robot_2 got robot_1's
        # arena_odom and its TF tree broke).
        placement = ({'robot_1': {'odom_origin_pose_id': 'midterm_robot_1',
                                  'parked_start_pose_id': '', 'rescue_straight_max_m': '1.0',
                                  'rescue_cooperative_accept_m': '0.10'},
                      'robot_2': {'odom_origin_pose_id': '',
                                  'parked_start_pose_id': 'midterm_robot_2',
                                  'rescue_straight_max_m': '1.0',
                                  'rescue_cooperative_accept_m': '0.10'}}
                     if midterm else {})
        if midterm:
            # SLAM needs robot_1/arena_odom before the stacks (and their identical copy of
            # this static transform) start after the map/TF gate below.
            start = resolve_parameters(mode, config).require(
                'localization.start_poses')['midterm_robot_1']
            actions.append(Node(
                package='tf2_ros', executable='static_transform_publisher',
                name='robot_1_arena_odom', output='screen', parameters=[{'use_sim_time': True}],
                arguments=['--x', str(start['x']), '--y', str(start['y']),
                           '--yaw', str(start['yaw']), '--frame-id', 'robot_1/arena_odom',
                           '--child-frame-id', 'robot_1/odom']))
        # Start only after observed map and TF readiness, independent of machine speed.
        stacks = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup / 'launch/robot_stack.launch.py')),
            launch_arguments={'robot_id': robot, 'run_mode': mode, 'use_sim_time': 'true',
                              'config_dir': str(config), 'require_competition_state': 'true',
                              'nav_mode': ('slam' if robot == 'robot_1' and not frozen_map
                                           else 'amcl'),
                              **placement.get(robot, {})}.items())
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
        DeclareLaunchArgument('scenario', default_value='',
                              description="'midterm': P4 arena, two robots, P3 and P4"),
        OpaqueFunction(function=setup),
    ])
