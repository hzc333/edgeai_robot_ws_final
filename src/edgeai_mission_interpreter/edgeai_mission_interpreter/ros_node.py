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

"""Asynchronous submit -> rule interpreter -> validator -> terminal intake."""

from collections import deque
import json
import time
from uuid import uuid4

from edgeai_rescue_coordinator.ros_support import ContextNode
from edgeai_robot_interfaces.msg import MissionIntakeResult, MissionPlan, MissionStep
from edgeai_robot_interfaces.msg import MissionValidationIssue
from edgeai_robot_interfaces.srv import SubmitMission, ValidateMission
import rclpy

from .interpreter import interpret, registry_from_target_list


class MissionInterpreter(ContextNode):
    """Receive first, then publish exactly one intake terminal per accepted request."""

    def __init__(self):
        super().__init__('mission_interpreter')
        self.queue = deque()
        self.pending = {}
        self.intake_pub = self.create_publisher(
            MissionIntakeResult, self.topic('mission_intake'), self.qos('event_stream'))
        self.validator = self.create_client(ValidateMission, self.service('validate_mission'))
        self.create_service(SubmitMission, self.service('submit_mission'), self.submit)
        self.create_timer(0.05, self.tick)

    def submit(self, request, response):
        response.accepted = True
        response.mission_id = 'mission-' + uuid4().hex
        response.message = 'received; interpretation and validation are asynchronous'
        intake = MissionIntakeResult(run_id=request.run_id, mission_id=response.mission_id,
                                     original_instruction=request.original_instruction,
                                     received_at=self.get_clock().now().to_msg(),
                                     model_name='rule-baseline', model_version='3cd021e',
                                     prompt_version='policy-v1')
        self.queue.append(intake)
        return response

    def finish(self, intake, outcome, issues):
        intake.header.stamp = self.get_clock().now().to_msg()
        intake.outcome = outcome
        intake.issues = issues
        self.intake_pub.publish(intake)

    def tick(self):
        while self.queue:
            intake = self.queue.popleft()
            try:
                result = interpret(intake.original_instruction,
                                   registry_from_target_list(self.targets))
                # Rule output is recorded verbatim as serialized structured output,
                # including policy notes; no model was called.
                intake.raw_model_output = json.dumps({
                    'steps': result.steps, 'notes': result.notes,
                    'issues': [{'code': i.code, 'detail': i.detail} for i in result.issues]},
                    ensure_ascii=False)
                if not result.ok:
                    self.finish(intake, 1, [MissionValidationIssue(code=i.code, detail=i.detail)
                                            for i in result.issues])
                    continue
                fields = dict(self.run_context, run_id=intake.run_id,
                              mission_id=intake.mission_id,
                              original_instruction=intake.original_instruction,
                              model_name=intake.model_name, model_version=intake.model_version,
                              prompt_version=intake.prompt_version, valid=False)
                plan = MissionPlan(**fields)
                plan.header.stamp = intake.received_at
                plan.header.frame_id = 'map'
                for step in result.steps:
                    fields = dict(step)
                    if fields.pop('action') != 'rescue':
                        raise ValueError('unsupported rule action')
                    plan.steps.append(MissionStep(action_type=MissionStep.ACTION_RESCUE, **fields))
                intake.candidate_plan = plan
                self.pending[intake.mission_id] = [intake, None, time.monotonic()]
            except Exception as exc:
                self.finish(intake, 1, [MissionValidationIssue(code=12, detail=str(exc))])
        for mid, (intake, future, started) in list(self.pending.items()):
            if future is not None and future.done():
                del self.pending[mid]
                try:
                    reply = future.result()
                    self.finish(intake, 3 if reply.accepted else 2, reply.issues)
                except Exception as exc:
                    self.finish(intake, 2, [MissionValidationIssue(code=0, detail=str(exc))])
            elif time.monotonic() - started > self.contract['timeouts_s']['validate_mission']:
                del self.pending[mid]
                if future is not None:
                    future.cancel()
                self.finish(intake, 2, [MissionValidationIssue(
                    code=0, detail='validation service timeout; remote outcome unknown')])
            elif future is None and self.validator.service_is_ready():
                self.pending[mid][1] = self.validator.call_async(
                    ValidateMission.Request(candidate=intake.candidate_plan))


def main(args=None):
    rclpy.init(args=args)
    node = MissionInterpreter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
