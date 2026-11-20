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

"""Validate against frozen targets and fresh authoritative runtime context."""

from copy import deepcopy
import time

from edgeai_robot_interfaces.msg import CompetitionState, MissionPlan, MissionValidationIssue
from edgeai_robot_interfaces.msg import RobotStateArray
from edgeai_robot_interfaces.srv import ValidateMission
import rclpy

from .ros_support import ContextNode, message, to_dict
from .validator import validate_mission


class MissionValidator(ContextNode):
    """The sole production publisher of validated mission plans."""

    def __init__(self):
        super().__init__('mission_validator')
        self.state = None
        self.robots = None
        self.state_at = self.robots_at = 0.0
        self.plan_pub = self.create_publisher(MissionPlan, self.topic('mission_plan'),
                                              self.qos('snapshot'))
        self.create_subscription(CompetitionState, self.topic('competition_state'),
                                 self.on_state, self.qos('control_state'))
        self.create_subscription(RobotStateArray, self.topic('robot_states'),
                                 self.on_robots, self.qos('robot_state'))
        self.create_service(ValidateMission, self.service('validate_mission'), self.validate)

    def on_state(self, msg):
        self.state, self.state_at = msg, time.monotonic()

    def on_robots(self, msg):
        self.robots, self.robots_at = msg, time.monotonic()

    def validate(self, request, response):
        now = time.monotonic()
        limits = self.contract['timeouts_s']
        if (self.state is None or self.robots is None or
                now - self.state_at > limits['competition_state_stale'] or
                now - self.robots_at > limits['robot_state_stale']):
            accepted, issues = False, [{'code': 110, 'step_id': '', 'target_id': '',
                                       'detail': 'runtime context unavailable or stale'}]
        else:
            state = self.state
            context = {key: getattr(state, key) for key in self.run_context}
            count = len({r.robot_id for r in self.robots.robots
                         if r.run_id == state.run_id and r.localized and
                         r.navigation_available and not r.faults})
            phase = state.phase if not state.estop_active and not state.reset_in_progress else -1
            accepted, issues = validate_mission(
                to_dict(request.candidate), self.targets, count, phase=phase, **context)
        response.accepted = accepted
        response.issues = [message(MissionValidationIssue, i) for i in issues]
        response.message = 'validated and published' if accepted else '; '.join(
            i['detail'] for i in issues)
        if accepted:
            plan = deepcopy(request.candidate)
            plan.valid = True
            self.plan_pub.publish(plan)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MissionValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
