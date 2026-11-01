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

"""Pure competition phase policy; ROS I/O and readiness live in the manager."""

from dataclasses import asdict, dataclass, field

PHASES = ('PREPARATION', 'TASK1', 'HANDOVER', 'TASK2A', 'FROZEN', 'TASK2B',
          'FINISHED', 'ESTOP')
MOTION_PHASES = {1, 3, 5}
SAFE_PHASES = {0, 2, 4, 6, 7}


@dataclass
class PhaseState:
    phase: int = 0
    sequence: int = 0
    interrupted: int = 255
    generation: int = 0
    frozen_from_task2a: bool = False
    reset_count: int = 0
    resetting_robot_ids: list = field(default_factory=list)

    def snapshot(self):
        return asdict(self)


class PhasePolicy:
    """Reject unauthorized edges and require explicit operator recovery from ESTOP."""

    def __init__(self, contract, state=None):
        self.contract = contract
        self.state = state or PhaseState()

    def destination(self, requested, requester):
        s = self.state
        if requester not in ('operator', 'system') or requested not in range(8):
            raise ValueError('invalid requester or phase')
        if requested == s.phase:
            return requested
        if requested == 7:
            return 7
        if s.phase == 7:
            if requester != 'operator':
                raise ValueError('only operator can release ESTOP')
            restored = s.interrupted if s.interrupted in (0, 4, 6) else 4
            if requested != restored:
                raise ValueError('ESTOP release cannot resume motion')
            return restored
        source = PHASES[s.phase]
        target = PHASES[requested]
        for edge in self.contract['phase_transitions']:
            if edge['from'] != source or requester not in edge['requesters']:
                continue
            if edge['to'] == 'RESUME_INTERRUPTED':
                if s.interrupted != 255 and requested == s.interrupted:
                    return requested
            elif edge['to'] == target:
                if edge.get('condition') == 'frozen_from_task2a_and_relocalized':
                    if not s.frozen_from_task2a:
                        continue
                return requested
        raise ValueError(f'illegal phase transition {source} -> {target}')

    def transition(self, requested, requester):
        destination = self.destination(requested, requester)
        s = self.state
        previous = s.phase
        if destination == previous:
            return False
        if destination == 7:
            # Repeated ESTOP must not lose the original interrupted phase.
            if previous != 4 or s.interrupted == 255:
                s.interrupted = previous
            s.generation += 1
        elif previous == 7:
            s.generation += 1
            if destination in (0, 6):
                s.interrupted = 255
        else:
            s.frozen_from_task2a = previous == 3 and destination == 4
            s.interrupted = 255
        s.phase = destination
        s.sequence += 1
        return True


def all_safe(robots, acknowledgements, run_id, sequence, phase, now, max_age):
    """Require fresh, exact acknowledgements, never vacuous success for no robots."""
    if not robots:
        return False
    for robot in robots:
        entry = acknowledgements.get(robot)
        if entry is None:
            return False
        ack, received = entry
        if now - received > max_age or received > now:
            return False
        if (ack.run_id != run_id or ack.robot_id != robot
                or ack.phase_sequence != sequence or ack.phase != phase
                or not ack.phase_received or not ack.safe or not ack.actions_cancelled
                or not ack.zero_velocity_commanded):
            return False
    return True
