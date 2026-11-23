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

"""Append-only rescue evidence and crash-safe completion receipts."""

import fcntl
import hashlib
import json
import os
from pathlib import Path


class Journal:
    """Refuse duplicate owners and mismatched rule/map reuse of a run directory."""

    def __init__(self, root, identity, *, completion_key=None):
        key = hashlib.sha256(identity['run_id'].encode()).hexdigest()[:24]
        self.directory = Path(root).expanduser() / key
        self.directory.mkdir(parents=True, exist_ok=True)
        self.lock = (self.directory / 'owner.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise
        self.identity = identity
        self.completion_key = completion_key or (lambda event: event['target_id'])
        path = self.directory / 'identity.json'
        if path.exists() and json.loads(path.read_text()) != identity:
            self.close()
            raise ValueError('same run_id has different map/configuration; use a new run')
        if not path.exists():
            self.atomic('identity.json', identity)
        self.previous = self.read_snapshot()
        self.sequence = 0
        self.completed = set()
        self.contacts = []
        self.monitor_started = None
        events_path = self.directory / 'events.jsonl'
        if events_path.exists():
            raw = events_path.read_bytes()
            lines = raw.splitlines(keepends=True)
            offset = 0
            for i, line in enumerate(lines):
                try:
                    event = json.loads(line)
                except (ValueError, UnicodeDecodeError):
                    if i != len(lines) - 1:
                        self.close()
                        raise ValueError('corrupt evidence before final record')
                    # A crash may leave only the final append incomplete.
                    with events_path.open('r+b') as stream:
                        stream.truncate(offset)
                    break
                self.sequence = max(self.sequence, event['seq'])
                if event['type'] == 'COMPLETED':
                    self.completed.add(self.completion_key(event))
                elif event['type'] == 'COLLISION':
                    self.contacts.append({k: event[k] for k in (
                        'sim_time', 'pair', 'sequence', 'physics_instance', 'physics_event_id')
                        if k in event})
                elif event['type'] == 'MONITOR_STARTED':
                    self.monitor_started = event['sim_time']
                offset += len(line)
            if raw and offset == len(raw) and not raw.endswith(b'\n'):
                with events_path.open('ab') as stream:
                    stream.write(b'\n')
        self.stream = events_path.open('a', encoding='utf-8', buffering=1)
        self.samples = (self.directory / 'distances.jsonl').open('a', encoding='utf-8')

    def atomic(self, name, value):
        path = self.directory / name
        temporary = path.with_suffix('.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(self.directory, os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

    def read_snapshot(self):
        path = self.directory / 'snapshot.json'
        return json.loads(path.read_text()) if path.exists() else None

    def event(self, event):
        if event['type'] == 'COMPLETED' and self.completion_key(event) in self.completed:
            return None
        self.sequence += 1
        event = dict(event, seq=self.sequence, run_id=self.identity['run_id'])
        event['event_id'] = f'{self.identity["run_id"]}:rescue:{self.sequence}'
        self.stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + '\n')
        self.stream.flush()
        os.fsync(self.stream.fileno())
        if event['type'] == 'COMPLETED':
            self.completed.add(self.completion_key(event))
        elif event['type'] == 'COLLISION':
            self.contacts.append({k: event[k] for k in (
                'sim_time', 'pair', 'sequence', 'physics_instance', 'physics_event_id')
                if k in event})
        elif event['type'] == 'MONITOR_STARTED':
            self.monitor_started = event['sim_time']
        return event

    def sample(self, sample):
        self.samples.write(json.dumps(sample, ensure_ascii=False, allow_nan=False) + '\n')
        self.samples.flush()

    def close(self):
        for name in ('stream', 'samples', 'lock'):
            stream = getattr(self, name, None)
            if stream is not None and not stream.closed:
                stream.close()
