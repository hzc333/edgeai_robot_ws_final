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

"""Exercise crash consistency, immutable identities and guarded phase edges."""

import copy
import json
from pathlib import Path
from types import SimpleNamespace

from edgeai_exploration.frontier import Grid
from edgeai_system_manager.freeze import encode_map, Task1Transaction
from edgeai_system_manager.phases import all_safe, PhasePolicy
from edgeai_system_manager.storage import EventJournal, RunStore, Schemas, utc_now
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
RUN = 'run-20261031T120000Z-12345678'


@pytest.fixture
def schemas():
    return Schemas(ROOT / 'schemas')


def test_run_identity_and_exclusive_writer(tmp_path):
    store = RunStore(tmp_path, RUN, {'parameters': 'a'})
    with pytest.raises(OSError):
        RunStore(tmp_path, RUN, {'parameters': 'a'})
    with pytest.raises(ValueError):
        store.path('../outside')
    store.write('system/state.json', {'phase': 2})
    store.close()
    with pytest.raises(ValueError):
        RunStore(tmp_path, RUN, {'parameters': 'b'})
    store = RunStore(tmp_path, RUN, {'parameters': 'a'})
    assert store.read('system/state.json')['phase'] == 2
    store.close()


def test_journal_restart_dedup_and_corruption(tmp_path, schemas):
    event = {'event_id': 'e1', 'ros_time_ns': 10, 'source_node': '/edge/system_manager',
             'phase': 'preparation', 'scoring_stage': 'none', 'category': 'operator',
             'event_type': 'started', 'payload': {'value': 1}}
    journal = EventJournal(tmp_path, RUN, schemas)
    assert journal.append(**event)['seq'] == 1
    assert journal.append(**event)['seq'] == 1
    journal.close()
    journal = EventJournal(tmp_path, RUN, schemas)
    assert journal.append(**event)['seq'] == 1
    with pytest.raises(ValueError):
        journal.append(**dict(event, payload={'value': 2}))
    journal.close()
    with (tmp_path / 'events.jsonl').open('ab') as stream:
        stream.write(b'{')
    with pytest.raises(ValueError, match='tail'):
        EventJournal(tmp_path, RUN, schemas)
    assert (tmp_path / 'events.jsonl').read_bytes().endswith(b'{')


def test_phase_permissions_estop_release_and_resume():
    contract = yaml.safe_load((ROOT / 'config/interface_runtime.yaml').read_text())
    policy = PhasePolicy(contract)
    with pytest.raises(ValueError):
        policy.transition(1, 'system')
    assert policy.transition(1, 'operator')
    assert not policy.transition(1, 'operator')
    policy.transition(7, 'system')
    generation = policy.state.generation
    policy.transition(7, 'system')
    assert policy.state.generation == generation
    with pytest.raises(ValueError):
        policy.transition(1, 'operator')
    with pytest.raises(ValueError):
        policy.transition(4, 'system')
    policy.transition(4, 'operator')
    assert policy.state.interrupted == 1
    policy.transition(1, 'operator')
    assert policy.state.interrupted == 255
    policy.transition(2, 'system')
    policy.transition(3, 'operator')
    policy.transition(4, 'system')
    policy.transition(5, 'operator')


def test_phase_ack_requires_identity_sequence_safety_and_freshness():
    ack = SimpleNamespace(run_id=RUN, robot_id='robot_1', phase_sequence=2, phase=2,
                          phase_received=True, safe=True, actions_cancelled=True,
                          zero_velocity_commanded=True)
    entries = {'robot_1': (ack, 10.0)}
    assert all_safe(['robot_1'], entries, RUN, 2, 2, 10.5, 1.0)
    assert not all_safe([], entries, RUN, 2, 2, 10.5, 1.0)
    assert not all_safe(['robot_1', 'robot_2'], entries, RUN, 2, 2, 10.5, 1.0)
    assert not all_safe(['robot_1'], entries, RUN, 3, 2, 10.5, 1.0)
    assert not all_safe(['robot_1'], entries, RUN, 2, 2, 12.0, 1.0)


def fixture_snapshot():
    grid = Grid(width=2, height=2, resolution=0.05, origin_x=-1.0,
                origin_y=-2.0, origin_yaw=0.0, data=(0, 100, -1, 0))
    targets = json.loads((ROOT / 'examples/interfaces/task1_targets.example.json').read_text())
    targets['run_id'] = RUN
    targets['map']['map_id'] = 'map-12345678'
    targets['map']['revision'] = 1
    manifest = {'schema_version': 1, 'run_id': RUN, 'manifest_revision': 1, 'updated_at': utc_now(),
                'run_mode': 'development', 'rulebook_version': 'V0-draft',
                'git': {'commit': 'a' * 40, 'dirty': True}, 'robots': ['robot_1'],
                'task1': None, 'resets': 0, 'files': []}
    return grid, targets, manifest


def test_pgm_has_correct_vertical_orientation():
    grid, _, _ = fixture_snapshot()
    image, metadata = encode_map(grid)
    assert image.endswith(bytes([205, 254, 254, 0]))
    assert yaml.safe_load(metadata)['origin'] == [-1.0, -2.0, 0.0]


def test_freeze_crash_recovery_idempotency_and_tampering(tmp_path, schemas, monkeypatch):
    store = RunStore(tmp_path, RUN, {})
    transaction = Task1Transaction(store, schemas)
    grid, targets, manifest = fixture_snapshot()
    original = store.write

    def fail_manifest(relative, value):
        if relative == 'manifest.json':
            raise OSError('simulated disk failure')
        original(relative, value)

    monkeypatch.setattr(store, 'write', fail_manifest)
    with pytest.raises(OSError):
        transaction.commit(grid, targets, manifest, 'map-12345678', 1, targets['revision'],
                           'team_declared_complete')
    assert not store.path('manifest.json').exists()
    monkeypatch.setattr(store, 'write', original)
    recovered = transaction.recover()
    assert recovered['task1']['map_revision'] == 1
    repeated = transaction.commit(grid, targets, manifest, 'map-12345678', 1,
                                  targets['revision'], 'team_declared_complete')
    assert repeated == recovered
    with pytest.raises(ValueError, match='different snapshot'):
        transaction.commit(grid, targets, manifest, 'map-12345678', 2,
                           targets['revision'], 'team_declared_complete')
    store.path('task1/map.pgm').write_bytes(b'corrupted')
    with pytest.raises(ValueError, match='changed'):
        transaction.recover()
    store.close()


def test_freeze_rejects_missing_map_and_mismatched_targets(tmp_path, schemas):
    store = RunStore(tmp_path, RUN, {})
    transaction = Task1Transaction(store, schemas)
    grid, targets, manifest = fixture_snapshot()
    for bad_grid, revision in [(None, targets['revision']), (grid, targets['revision'] + 1)]:
        with pytest.raises(ValueError):
            transaction.commit(bad_grid, targets, manifest, 'map-12345678', 1, revision,
                               'team_declared_complete')
    bad = copy.deepcopy(targets)
    bad['targets'][0]['marker_pose']['orientation']['w'] = 0.0
    with pytest.raises(ValueError, match='quaternion'):
        transaction.commit(grid, bad, manifest, 'map-12345678', 1, targets['revision'],
                           'team_declared_complete')
    assert not store.path('manifest.json').exists()
    assert not store.path('task1').exists()
    store.close()
