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
Start Nav2 (and optionally AMCL) for one robot in its namespace.

Unlike nav2_bringup's navigation_launch.py, /tf and /tf_static are NOT
remapped into the namespace: all robots share the global TF tree with
robot_N/ frame prefixes (ADR-0002). Velocity flows controller -> cmd_vel_nav
-> velocity_smoother -> cmd_vel_smoothed -> collision_monitor ->
cmd_vel_safe_in -> safety_watchdog (ADR-0003).

mode:=amcl  localize on the shared /map with AMCL (Task 2)
mode:=slam  SLAM provides map -> robot_N/odom (Task 1, P2); AMCL and the
            localization node are started inactive for the handover

Nav2 can only activate once map -> robot_N/base_footprint exists; otherwise the
costmaps time out and the lifecycle manager aborts the bring-up for good. In
amcl mode neither AMCL nor the navigation nodes start automatically: the
localization node starts AMCL and then navigation on the first successful
/<robot_id>/relocalize call, e.g.
  ros2 service call /robot_1/relocalize edgeai_robot_interfaces/srv/Relocalize \
    "{mode: 2, start_pose_id: task1_start}"
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from edgeai_common.config import find_config_dir, resolve_parameters
from edgeai_common.frames import validate_robot_id
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import yaml

NAVIGATION_NODES = ['controller_server', 'smoother_server', 'planner_server',
                    'behavior_server', 'velocity_smoother', 'collision_monitor',
                    'bt_navigator']


def _as_bool(text):
    return text.strip().lower() in ('true', '1', 'yes')


def render_params(template, robot_id, max_linear, max_angular, bt_dir=''):
    """
    Fill the placeholders of nav2_sim.yaml and nest it under the robot namespace.

    ROS 2 matches parameter-file keys against fully qualified node names, so the
    parameters of /robot_1/controller_server must sit under robot_1:
    controller_server:. Without the namespace root the nodes silently fall back
    to their defaults.
    """
    text = (template.replace('<ROBOT>', robot_id)
            .replace('<MAX_LINEAR>', repr(float(max_linear)))
            .replace('<MAX_ANGULAR>', repr(float(max_angular)))
            .replace('<SLOW_LINEAR>', repr(0.5 * float(max_linear)))
            .replace('<SLOW_ANGULAR>', repr(0.5 * float(max_angular)))
            .replace('<BT_DIR>', bt_dir))
    return yaml.safe_dump({robot_id: yaml.safe_load(text)}, sort_keys=False)


def _launch_setup(context, *args, **kwargs):
    robot_id = validate_robot_id(LaunchConfiguration('robot_id').perform(context))
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('amcl', 'slam'):
        raise RuntimeError(f'mode must be amcl or slam, got {mode!r}')
    run_mode = LaunchConfiguration('run_mode').perform(context)
    config_dir = find_config_dir(LaunchConfiguration('config_dir').perform(context) or None)
    use_sim_time = _as_bool(LaunchConfiguration('use_sim_time').perform(context))
    autostart_localization = _as_bool(
        LaunchConfiguration('autostart_localization').perform(context))
    require_state = _as_bool(LaunchConfiguration('require_competition_state').perform(context))
    if mode == 'slam' and autostart_localization:
        # Two map -> robot_N/odom publishers (SLAM and AMCL) are forbidden (ADR-0002).
        raise RuntimeError('autostart_localization must be false in slam mode')
    parked = LaunchConfiguration('parked_start_pose_id').perform(context)
    origin_id = LaunchConfiguration('odom_origin_pose_id').perform(context)
    if parked and mode != 'amcl':
        raise RuntimeError('parked_start_pose_id needs mode:=amcl (AMCL takes over when '
                           'the parked robot is relocalized)')
    autostart_navigation_text = LaunchConfiguration('autostart_navigation').perform(context)
    autostart_navigation = (mode == 'slam' if autostart_navigation_text == 'auto'
                            else _as_bool(autostart_navigation_text))

    resolved = resolve_parameters(run_mode, config_dir)
    share_config = os.path.join(get_package_share_directory('edgeai_robot_navigation'), 'config')
    template_path = LaunchConfiguration('params_file').perform(context) or os.path.join(
        share_config, 'nav2_sim.yaml')
    with open(template_path, encoding='utf-8') as stream:
        rendered = render_params(stream.read(), robot_id,
                                 resolved.require('motion.max_linear_speed_mps'),
                                 resolved.require('motion.max_angular_speed_radps'),
                                 share_config)
    start_poses = resolved.get('localization.start_poses', {}) or {}
    origin = None
    if origin_id:
        # SLAM and AMCL localize <robot>/arena_odom, a fixed copy of the odometry frame
        # moved to the arena origin, so the map is the arena frame from the start.
        if origin_id not in start_poses:
            raise RuntimeError(f'odom_origin_pose_id {origin_id!r} is not a start pose')
        origin = start_poses[origin_id]
        data = yaml.safe_load(rendered)
        data[robot_id]['amcl']['ros__parameters']['odom_frame_id'] = f'{robot_id}/arena_odom'
        rendered = yaml.safe_dump(data, sort_keys=False)
    params_file = os.path.join(tempfile.mkdtemp(prefix=f'edgeai_{robot_id}_nav2_'), 'nav2.yaml')
    with open(params_file, 'w', encoding='utf-8') as stream:
        stream.write(rendered)

    common = {'use_sim_time': use_sim_time}
    footprint = yaml.safe_load(rendered)[robot_id]['local_costmap']['local_costmap'][
        'ros__parameters']['footprint']

    def nav2_node(package, executable, name=None, remappings=()):
        return Node(package=package, executable=executable, name=name or executable,
                    output='screen', parameters=[params_file, common],
                    remappings=list(remappings))

    nodes = [
        nav2_node('nav2_controller', 'controller_server',
                  remappings=[('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_smoother', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', remappings=[('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_velocity_smoother', 'velocity_smoother',
                  remappings=[('cmd_vel', 'cmd_vel_nav')]),
        nav2_node('nav2_collision_monitor', 'collision_monitor'),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        # The other robots are below the LiDAR plane: obstacles from the shared /tf.
        Node(package='edgeai_robot_navigation', executable='peer_footprints',
             name='peer_footprints', output='screen',
             parameters=[common, {'robot_id': robot_id, 'footprint': footprint}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[common, {'autostart': autostart_navigation,
                                  'node_names': NAVIGATION_NODES}]),
    ]
    # AMCL and the localization node also run (inactive) in slam mode so that the
    # SLAM -> AMCL handover (ADR-0002 appendix) is a single relocalize call.
    nodes += [
        nav2_node('nav2_amcl', 'amcl'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[common, {'autostart': autostart_localization,
                                  'node_names': ['amcl']}]),
        Node(package='edgeai_robot_navigation', executable='localization',
             name='localization', output='screen',
             parameters=[common, {
                 'robot_id': robot_id,
                 'run_mode': run_mode,
                 'config_dir': str(config_dir),
                 'require_competition_state': require_state,
                 'start_poses_yaml': yaml.safe_dump(start_poses),
                 # Its first relocalization is the SLAM -> AMCL handover.
                 'slam_handover': mode == 'slam',
                 'parked_start_pose_id': parked,
                 'parked_map_origin_pose_id':
                     LaunchConfiguration('parked_map_origin_pose_id').perform(context),
             }]),
    ]
    if origin is not None:
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='arena_odom_publisher', output='screen', parameters=[common],
            arguments=['--x', str(origin['x']), '--y', str(origin['y']),
                       '--yaw', str(origin['yaw']), '--frame-id', f'{robot_id}/arena_odom',
                       '--child-frame-id', f'{robot_id}/odom']))
    return [GroupAction([PushRosNamespace(robot_id), *nodes])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot_1'),
        DeclareLaunchArgument('mode', default_value='amcl', description='amcl | slam'),
        DeclareLaunchArgument('run_mode', default_value='development'),
        DeclareLaunchArgument('config_dir', default_value='',
                              description='Repository config directory (default: search)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 parameter template (default: nav2_sim.yaml)'),
        DeclareLaunchArgument('autostart_navigation', default_value='auto',
                              description='auto: true for slam, false for amcl (activate '
                                          'after localization)'),
        DeclareLaunchArgument('parked_start_pose_id', default_value='',
                              description='amcl mode: publish map -> odom from this start '
                                          'pose until the first relocalization'),
        DeclareLaunchArgument('parked_map_origin_pose_id', default_value='',
                              description='Start pose at which the SLAM map is anchored '
                                          '(empty: map is the arena frame)'),
        DeclareLaunchArgument('odom_origin_pose_id', default_value='',
                              description='Start pose of this robot in the arena: SLAM and '
                                          'AMCL then use <robot>/arena_odom so that map is '
                                          'the arena frame (SLAM ignores map_start_pose)'),
        DeclareLaunchArgument('autostart_localization', default_value='false',
                              description='Activate AMCL immediately instead of on the '
                                          'first relocalize call'),
        DeclareLaunchArgument('require_competition_state', default_value='true',
                              description='false only in development: relocalize without '
                                          'checking run_id and map version'),
        OpaqueFunction(function=_launch_setup),
    ])
