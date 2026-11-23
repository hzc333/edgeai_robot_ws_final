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

"""Behavioral verification of official geometry and asynchronous failure races."""

from dataclasses import replace
import json
import math
from pathlib import Path

from edgeai_task_coordinator.engine import Engine
from edgeai_task_coordinator.evidence import Journal
from edgeai_task_coordinator.model import Configuration, load_configuration, Robot
from edgeai_task_coordinator.planning import allocate, GridCost
from edgeai_task_coordinator.verification import DwellVerifier
import pytest
import yaml


CONFIG = Path(__file__).parents[1] / 'config' / 'midterm.yaml'


@pytest.fixture
def config():
    return load_configuration(CONFIG)


def samples(target, stamp, *, task='', mode=4):
    return [Robot(f'robot_{i + 1}', stamp, *slot[:2], mode=mode, active_task=task)
            for i, slot in enumerate(target.slots)]


@pytest.mark.parametrize('field,value', [
    ('radius_m', 0.15), ('dwell_s', 3.0), ('marker_id', 10),
    ('center', [0.0, 0.0]), ('required_robots', 2),
])
def test_official_rule_changes_are_rejected(tmp_path, field, value):
    data = yaml.safe_load(CONFIG.read_text())
    data['targets'][0][field] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='official midterm'):
        load_configuration(path)


@pytest.mark.parametrize('value', [float('nan'), -1, 0, float('inf')])
def test_bad_engineering_limits_rejected(tmp_path, value):
    data = yaml.safe_load(CONFIG.read_text())
    data['engineering']['state_stale_s'] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_configuration(path)


@pytest.mark.parametrize('index', [0, 1])
def test_continuous_ten_seconds_and_pause(config, index):
    target = config.targets[index]
    verifier = DwellVerifier()
    for i in range(100):
        now = 100 + i / 10
        complete, _, _ = verifier.update(now, samples(target, now), target, config.settings)
        assert not complete
    for _ in range(50):  # paused simulation: callbacks do not create extra dwell
        assert not verifier.update(now, samples(target, now), target, config.settings)[0]
    assert verifier.update(110, samples(target, 110), target, config.settings)[0]


def test_any_partner_exit_resets_the_common_timer(config):
    target = config.targets[1]
    verifier = DwellVerifier()
    for i in range(91):
        now = 1 + i / 10
        verifier.update(now, samples(target, now), target, config.settings)
    outside = samples(target, 10.1)
    outside[1] = replace(outside[1], x=target.center[0] + target.radius + 0.001,
                         y=target.center[1])
    assert verifier.update(10.1, outside, target, config.settings)[1]
    assert verifier.elapsed == 0
    for i in range(100):
        now = 11 + i / 10
        assert not verifier.update(now, samples(target, now), target, config.settings)[0]
    assert verifier.update(21, samples(target, 21), target, config.settings)[0]


def test_judgement_is_target_centred_not_stop_centred(config):
    target = config.targets[0]
    verifier = DwellVerifier()
    # Opposite side of the legal circle: far from the selected navigation goal.
    robot = Robot('robot_1', 1, 1.0, 3.61, mode=4)
    verifier.update(1, [robot], target, config.settings)
    assert verifier.start == 1
    assert math.dist((robot.x, robot.y), target.slots[0][:2]) > 0.8


def test_distinct_zone_dwell_resets_if_partners_swap_zones(config):
    target = replace(config.targets[1], radius=0.15, dwell=1.0)
    zones = [slot[:2] for slot in target.slots]
    verifier = DwellVerifier()
    robots = [Robot(f'robot_{i + 1}', 1.0, *xy) for i, xy in enumerate(zones)]
    assert not verifier.update(1.0, robots, target, config.settings, zones=zones)[0]
    swapped = [replace(r, stamp=1.2, x=zones[1 - i][0], y=zones[1 - i][1])
               for i, r in enumerate(robots)]
    assert verifier.update(1.2, swapped, target, config.settings, zones=zones)[1]
    assert verifier.start is None
    for i in range(6):
        now = 1.4 + i / 5
        result = verifier.update(now, [replace(r, stamp=now) for r in robots],
                                 target, config.settings, zones=zones)
        assert result[0] == (i == 5)


@pytest.mark.parametrize('failure', ['stale', 'skew', 'duplicate', 'nan', 'unlocalized'])
def test_unreliable_pair_never_counts(config, failure):
    target = config.targets[1]
    verifier = DwellVerifier()
    robots = samples(target, 1)
    verifier.update(1, robots, target, config.settings)
    robots = samples(target, 1.1)
    if failure == 'stale':
        robots[0] = replace(robots[0], stamp=0.1)
    elif failure == 'skew':
        robots[0] = replace(robots[0], stamp=0.8)
    elif failure == 'duplicate':
        robots[1] = replace(robots[1], robot_id=robots[0].robot_id)
    elif failure == 'nan':
        robots[0] = replace(robots[0], x=float('nan'))
    else:
        robots[0] = replace(robots[0], localized=False)
    assert verifier.update(1.1, robots, target, config.settings)[1]
    assert verifier.start is None


def test_cached_samples_gap_and_clock_rewind(config):
    target = config.targets[0]
    verifier = DwellVerifier()
    verifier.update(100, samples(target, 100), target, config.settings)
    for now in (100.1, 100.2, 100.3):
        verifier.update(now, samples(target, 100), target, config.settings)
        assert verifier.elapsed == 0
    verifier.update(101, samples(target, 101), target, config.settings)
    assert verifier.start == 101  # unseen interval cannot be credited
    verifier.update(1, samples(target, 1), target, config.settings)
    assert verifier.start is None


def test_grid_route_avoids_wall_and_corner_cutting():
    width, height = 12, 12
    data = [0] * (width * height)
    for y in range(10):
        data[y * width + 5] = 100
    grid = GridCost(width, height, 1.0, (0, 0, 0), data, clearance=0)
    assert grid((3.5, 2.5), (7.5, 2.5)) > 16
    assert math.isinf(grid((5.5, 2.5), (7.5, 2.5)))
    blocked = GridCost(2, 2, 1, (0, 0, 0), [0, 100, 100, 0], clearance=0)
    assert math.isinf(blocked((0.5, 0.5), (1.5, 1.5)))


def test_assignment_checks_reachability_and_distinct_robots():
    robots = {'robot_1': Robot('robot_1', 1, 0, 0),
              'robot_2': Robot('robot_2', 1, 10, 0)}
    assert allocate(robots, [(9, 0, 0), (1, 0, 0)], math.dist) == ('robot_2', 'robot_1')
    assert allocate({'robot_1': robots['robot_1']}, [(1, 0, 0), (2, 0, 0)], math.dist) is None
    assert allocate(robots, [(1, 0, 0)], lambda *_: math.inf) is None


def start_engine(config, cooperative=False):
    target = config.targets[1 if cooperative else 0]
    engine = Engine(Configuration(config.robots, (target,), config.settings), 'run-test')
    robots = {rid: Robot(rid, 1, i, 0) for i, rid in enumerate(config.robots)}
    engine.tick(1, robots, True, math.dist)
    return engine, robots


def test_partner_is_reserved_and_sent_only_after_first_holds(config):
    engine, robots = start_engine(config, cooperative=True)
    commands = engine.drain_commands()
    assert len(commands) == 1 and len(engine.assignment) == 2
    first = commands[0]
    engine.accepted(first.token, True, 1)
    robots = {r: replace(s, stamp=1.1) for r, s in robots.items()}
    engine.tick(1.1, robots, True, math.dist)
    assert engine.drain_commands() == []
    robots[first.robot_id] = replace(
        robots[first.robot_id], mode=4, active_task=engine.task_id,
        x=engine.target.slots[0][0], y=engine.target.slots[0][1], yaw=-math.pi / 2)
    engine.tick(1.1, robots, True, math.dist)
    assert not engine.drain_commands()
    robots = {r: replace(s, stamp=1.7) for r, s in robots.items()}
    engine.tick(1.7, robots, True, math.dist)
    second = engine.drain_commands()[0]
    assert second.robot_id != first.robot_id
    engine.accepted(second.token, False, 1.1)
    assert engine.status == 'RELEASING'
    assert any(c.kind == 'cancel' and c.token == first.token for c in engine.drain_commands())


def test_late_accept_after_timeout_is_cancelled_not_reassigned(config):
    engine, robots = start_engine(config)
    command = engine.drain_commands()[0]
    now = 1 + config.settings.action_response_timeout_s + 0.1
    robots = {r: replace(s, stamp=now) for r, s in robots.items()}
    engine.tick(now, robots, True, math.dist)
    assert engine.status == 'RELEASING'
    engine.accepted(command.token, True, now)
    assert any(c.kind == 'cancel' and c.token == command.token for c in engine.drain_commands())


def test_completion_survives_intentional_cancel_and_cannot_repeat(config):
    engine, robots = start_engine(config)
    command = engine.drain_commands()[0]
    engine.accepted(command.token, True, 1)
    for i in range(107):
        now = 2 + i / 10
        robots = {r: replace(s, stamp=now) for r, s in robots.items()}
        target = engine.target
        robots[command.robot_id] = replace(
            robots[command.robot_id], x=target.center[0], y=target.center[1],
            active_task=engine.task_id, mode=4)
        engine.tick(now, robots, True, math.dist)
    assert engine.status == 'RELEASING'
    assert sum(e['type'] == 'COMPLETED' for e in engine.events) == 1
    engine.ended(command.token, now)
    robots = {r: replace(s, stamp=now + 0.1, active_task='', mode=1) for r, s in robots.items()}
    engine.tick(now + 0.1, robots, True, math.dist)
    engine.tick(now + 0.2, robots, True, math.dist)
    assert engine.status == 'DONE'
    assert engine.completed == {'S1'}
    assert sum(e['type'] == 'COMPLETED' for e in engine.events) == 1


def test_freeze_cancels_until_adapter_authorizes_recovery(config):
    engine, robots = start_engine(config)
    command = engine.drain_commands()[0]
    engine.accepted(command.token, True, 1)
    engine.tick(1.1, robots, False, math.dist)
    assert engine.status == 'RELEASING'
    engine.ended(command.token, 1.1)
    engine.tick(1.2, robots, True, math.dist)
    assert engine.status == 'RECOVERING'
    engine.resume(1.3)
    assert engine.status == 'WAITING'
    assert engine.verifier.elapsed == 0


def test_navigation_through_circle_is_not_an_early_admission(config):
    engine, robots = start_engine(config)
    command = engine.drain_commands()[0]
    engine.accepted(command.token, True, 1)
    for i in range(120):
        now = 2 + i / 10
        robots = {r: replace(s, stamp=now) for r, s in robots.items()}
        robots[command.robot_id] = replace(
            robots[command.robot_id], x=1.0, y=4.1, mode=3, active_task=engine.task_id)
        engine.tick(now, robots, True, math.dist)
    assert not engine.completed
    assert engine.verifier.start is None


def test_release_requires_both_terminal_action_and_idle_state(config):
    engine, robots = start_engine(config)
    command = engine.drain_commands()[0]
    engine.accepted(command.token, True, 1)
    engine.fail('TEST_FAILURE', 1.1, retry=True)
    engine.ended(command.token, 1.2)
    robots[command.robot_id] = replace(
        robots[command.robot_id], stamp=1.2, mode=4, active_task=engine.task_id)
    engine.tick(1.2, robots, True, math.dist)
    assert engine.status == 'RELEASING'
    engine.tick(1.1 + config.settings.release_timeout_s + 0.1, robots, True, math.dist)
    assert engine.status == 'HALTED'
    assert engine.attempts['S1'] == 1


def test_old_result_token_cannot_fail_a_new_attempt(config):
    engine, robots = start_engine(config)
    old = engine.drain_commands()[0]
    engine.accepted(old.token, False, 1.1)
    robots = {r: replace(s, stamp=1.2) for r, s in robots.items()}
    engine.tick(1.2, robots, True, math.dist)
    robots = {r: replace(s, stamp=2.2) for r, s in robots.items()}
    engine.tick(2.2, robots, True, math.dist)
    assert engine.status == 'ACTIVE'
    assert engine.attempts['S1'] == 2
    engine.ended(old.token, 1.3)
    assert engine.status == 'ACTIVE'


def test_durable_completion_and_run_identity(tmp_path):
    identity = {'run_id': 'run-1', 'map_id': 'map-1'}
    journal = Journal(tmp_path, identity)
    assert journal.event({'type': 'COMPLETED', 'target_id': 'S1'})
    assert journal.event({'type': 'COMPLETED', 'target_id': 'S1'}) is None
    journal.close()
    journal = Journal(tmp_path, identity)
    assert journal.completed == {'S1'}
    journal.close()
    with pytest.raises(ValueError, match='different map'):
        Journal(tmp_path, dict(identity, map_id='map-2'))


def test_evidence_lock_and_partial_append_recovery(tmp_path):
    identity = {'run_id': 'run-1'}
    journal = Journal(tmp_path, identity)
    with pytest.raises(BlockingIOError):
        Journal(tmp_path, identity)
    journal.event({'type': 'ASSIGNED', 'target_id': 'S1'})
    directory = journal.directory
    journal.close()
    with (directory / 'events.jsonl').open('ab') as stream:
        stream.write(b'{"seq": 2,')
    journal = Journal(tmp_path, identity)
    journal.event({'type': 'COMPLETED', 'target_id': 'S1'})
    journal.close()
    lines = (directory / 'events.jsonl').read_text().splitlines()
    assert [json.loads(s)['seq'] for s in lines] == [1, 2]


def test_prepared_distance_fields_match_astar_without_corner_cutting():
    data = [0] * 144
    for y in range(10):
        data[y * 12 + 5] = 100
    grid = GridCost(12, 12, 1.0, (0, 0, 0), data, clearance=0)
    goal = (7.5, 2.5)
    starts = [(3.5, 2.5), (2.5, 10.5), (5.5, 2.5)]
    expected = [grid(start, goal) for start in starts]
    grid.prepare([goal])
    assert [grid(start, goal) for start in starts] == pytest.approx(expected)


def test_already_parked_robot_can_resume_inside_conservative_allocation_clearance():
    robots = {'robot_1': Robot('robot_1', 1, 1.0, 4.42)}
    # Localization can put the parked center inside allocation inflation while
    # its real footprint remains clear. No navigation is required at this slot.
    assert allocate(robots, [(1.0, 4.45, 0)], lambda *_: math.inf,
                    [(0.55, 4.45, 0)], parking_tolerance=0.06) == ('robot_1',)
    assert allocate(robots, [(1.0, 4.45, 0)], lambda *_: math.inf,
                    [(0.55, 4.45, 0)], parking_tolerance=0.01) is None


def test_phase_result_race_does_not_consume_navigation_retry_budget(config):
    engine, robots = start_engine(config)
    command = engine.drain_commands()[0]
    engine.accepted(command.token, True, 1)
    engine.ended(command.token, 1.1, 'phase change result arrived first')
    assert engine.failures['S1'] == 1
    engine.interrupt('PHASE_OR_SAFETY_STOP', 1.1)
    engine.interrupt('PHASE_OR_SAFETY_STOP', 1.2)
    assert engine.failures['S1'] == 0
    engine.tick(1.2, robots, False, math.dist)
    assert engine.status == 'RECOVERING'
