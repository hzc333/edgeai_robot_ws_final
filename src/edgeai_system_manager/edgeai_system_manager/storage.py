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

"""Crash-safe files, validated append-only events, and exclusive run ownership."""

from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import uuid

from jsonschema import Draft202012Validator, FormatChecker
from referencing import Registry, Resource


def utc_now():
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def new_run_id():
    return datetime.now(timezone.utc).strftime('run-%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:8]


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def sync_directory(path):
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def atomic_bytes(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name('.' + path.name + '.' + uuid.uuid4().hex + '.tmp')
    try:
        with temporary.open('xb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        sync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path, value):
    atomic_bytes(path, (json.dumps(value, ensure_ascii=False, allow_nan=False,
                                   indent=2, sort_keys=True) + '\n').encode())


class Schemas:
    """Validate output with the repository's versioned schemas and date formats."""

    def __init__(self, directory):
        documents = [json.loads(path.read_text())
                     for path in Path(directory).glob('*.schema.json')]
        registry = Registry().with_resources(
            (doc['$id'], Resource.from_contents(doc)) for doc in documents)
        self.validators = {}
        for path in Path(directory).glob('*.schema.json'):
            schema = json.loads(path.read_text())
            Draft202012Validator.check_schema(schema)
            self.validators[path.name.removesuffix('.schema.json')] = Draft202012Validator(
                schema, registry=registry, format_checker=FormatChecker())

    def check(self, name, document):
        # JSON must reject non-finite numbers even where jsonschema accepts them.
        json.dumps(document, allow_nan=False)
        self.validators[name].validate(document)
        return document


class RunStore:
    """Single writer for a run; reopening requires the same explicit identity.

    A workspace-level lease prevents two managers using different run IDs from
    concurrently controlling one deployment. No automatic new run on restart.
    """

    def __init__(self, root, run_id, identity):
        if not re.fullmatch(r'run-\d{8}T\d{6}Z-[a-f0-9]{8}', run_id):
            raise ValueError('invalid run_id')
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.lease = (self.root / '.manager.lock').open('a')
        try:
            fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.directory = self.root / run_id
            self.directory.mkdir(exist_ok=True)
            path = self.directory / 'system/identity.json'
            expected = dict(identity, run_id=run_id)
            if path.exists():
                if json.loads(path.read_text()) != expected:
                    raise ValueError('run identity/configuration mismatch')
            else:
                atomic_json(path, expected)
            self.run_id = run_id
        except Exception:
            self.close()
            raise

    def path(self, relative):
        path = (self.directory / relative).resolve()
        if path == self.directory or self.directory not in path.parents:
            raise ValueError('path escapes run directory')
        return path

    def read(self, relative, default=None):
        path = self.path(relative)
        return json.loads(path.read_text()) if path.exists() else default

    def write(self, relative, value):
        atomic_json(self.path(relative), value)

    def close(self):
        if not self.lease.closed:
            fcntl.flock(self.lease, fcntl.LOCK_UN)
            self.lease.close()


class EventJournal:
    """Sole events.jsonl writer, with schema checks and durable event-ID deduplication."""

    def __init__(self, directory, run_id, schemas):
        self.path = Path(directory) / 'events.jsonl'
        self.run_id = run_id
        self.schemas = schemas
        self.sequence = 0
        self.ids = {}
        self.lock = self.path.with_suffix('.lock').open('a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if self.path.exists():
                raw = self.path.read_bytes()
                # Never silently truncate a crash tail: preserve evidence and refuse
                # restart until an explicit audit/recovery action has handled it.
                if raw and not raw.endswith(b'\n'):
                    raise ValueError('incomplete event journal tail; recovery required')
                for line in raw.splitlines():
                    record = json.loads(line)
                    schemas.check('event_record', record)
                    if record['run_id'] != run_id or record['seq'] != self.sequence + 1:
                        raise ValueError('event identity or sequence mismatch')
                    if record['event_id'] in self.ids:
                        raise ValueError('duplicate event ID on disk')
                    self.sequence += 1
                    self.ids[record['event_id']] = record
            self.stream = self.path.open('ab', buffering=0)
        except Exception:
            self.lock.close()
            raise

    def append(self, *, event_id, ros_time_ns, source_node, phase, scoring_stage,
               category, event_type, payload):
        record = {'schema_version': 1, 'run_id': self.run_id, 'seq': self.sequence + 1,
                  'event_id': event_id, 'ros_time_ns': ros_time_ns, 'wall_time': utc_now(),
                  'source_node': source_node, 'phase': phase, 'scoring_stage': scoring_stage,
                  'category': category, 'type': event_type, 'payload': payload}
        self.schemas.check('event_record', record)
        if event_id in self.ids:
            old = self.ids[event_id]
            ignored = {'seq', 'wall_time'}
            if {k: v for k, v in old.items() if k not in ignored} != {
                    k: v for k, v in record.items() if k not in ignored}:
                raise ValueError('event ID reused with different content')
            return old
        data = (json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n').encode()
        self.stream.write(data)
        os.fsync(self.stream.fileno())
        self.sequence += 1
        self.ids[event_id] = record
        return record

    def close(self):
        self.stream.close()
        self.lock.close()
