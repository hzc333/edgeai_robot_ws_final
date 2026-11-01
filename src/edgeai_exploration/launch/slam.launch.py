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

"""SLAM Toolbox lifecycle with the P2-managed input map topic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    slam = LifecycleNode(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        namespace='', output='screen',
        parameters=[LaunchConfiguration('slam_params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'use_lifecycle_manager': False}],
        remappings=[('map', '/edge/slam_map'), ('map_metadata', '/edge/slam_map_metadata')])
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'), activate, slam,
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(slam),
                                    transition_id=Transition.TRANSITION_CONFIGURE))])
