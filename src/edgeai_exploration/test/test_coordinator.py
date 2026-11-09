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

"""Post-exploration return to the start area: a failed return is retried."""

from types import SimpleNamespace
from unittest.mock import Mock

from edgeai_exploration.coordinator import ExplorationCoordinator

HOME = ('home', 'midterm_robot_1', 0.5, 0.5, None)


def _coordinator(goal, retries=3, cancelling=False):
    parameters = {'return_retries': retries, 'viewpoint_dwell_s': 2.0}
    return SimpleNamespace(
        extra_goal=goal, extra=[], home_retries=0, cancelling=cancelling, goal_handle=object(),
        dwell_until=None, get_logger=Mock,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]))


def _end(coordinator, success):
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        result=SimpleNamespace(success=success)))
    ExplorationCoordinator.completed(coordinator, future)


def test_failed_return_home_is_queued_again_until_the_retries_run_out():
    coordinator = _coordinator(HOME, retries=2)
    for attempt in (1, 2):
        _end(coordinator, False)
        assert coordinator.extra == [HOME] and coordinator.home_retries == attempt
        assert coordinator.goal_handle is None
        coordinator.extra.pop(0)            # after_exploration sends it again
    _end(coordinator, False)
    assert coordinator.extra == []          # given up: Task 1 ends as before


def test_reached_home_and_cancelled_home_are_not_retried():
    coordinator = _coordinator(HOME)
    _end(coordinator, True)
    assert coordinator.extra == [] and coordinator.home_retries == 0
    cancelled = _coordinator(HOME, cancelling=True)
    _end(cancelled, False)
    assert cancelled.extra == [] and not cancelled.cancelling


def test_failed_viewpoint_is_not_retried():
    coordinator = _coordinator(('view', 'S1', 1.6, 3.76, 2.62))
    _end(coordinator, False)
    assert coordinator.extra == [] and coordinator.dwell_until is None
