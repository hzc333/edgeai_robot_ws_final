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

"""Read-only run audit and reproducible export of verified manifest artifacts."""

import argparse
import json
from pathlib import Path
import tarfile

import yaml

from .freeze import validate_targets
from .storage import Schemas, sha256


def audit(directory, schema_directory, require_finished=False):
    root = Path(directory).resolve()
    schemas = Schemas(schema_directory)
    manifest = json.loads((root / 'manifest.json').read_text())
    schemas.check('manifest', manifest)
    if manifest['run_id'] != root.name:
        raise ValueError('run directory identity mismatch')
    seen = set()
    for item in manifest['files']:
        path = (root / item['path']).resolve()
        if root not in path.parents or item['path'] in seen:
            raise ValueError('invalid or duplicate manifest path')
        seen.add(item['path'])
        if sha256(path) != item['sha256'] or path.stat().st_size != item['size_bytes']:
            raise ValueError('file hash/size mismatch: ' + item['path'])
    events_path = root / 'events.jsonl'
    events, event_ids = [], set()
    if events_path.exists():
        raw = events_path.read_bytes()
        if raw and not raw.endswith(b'\n'):
            raise ValueError('incomplete journal')
        for line in raw.splitlines():
            record = json.loads(line)
            schemas.check('event_record', record)
            if (record['seq'] != len(events) + 1 or record['event_id'] in event_ids
                    or record['run_id'] != manifest['run_id']):
                raise ValueError('invalid event ordering/identity')
            events.append(record)
            event_ids.add(record['event_id'])
    if manifest['task1'] is not None:
        target_doc = json.loads((root / 'task1/targets.json').read_text())
        schemas.check('target_list', target_doc)
        validate_targets(target_doc)
        metadata = target_doc['map']
        map_yaml = yaml.safe_load((root / metadata['yaml']).read_text())
        expected_origin = [metadata['origin'][key] for key in ('x', 'y', 'yaw')]
        if (map_yaml['resolution'] != metadata['resolution_m']
                or map_yaml['origin'] != expected_origin):
            raise ValueError('map YAML/metadata mismatch')
        if (sha256(root / metadata['yaml']) != metadata['yaml_sha256']
                or sha256(root / metadata['image']) != metadata['image_sha256']):
            raise ValueError('target list references a different map')
    if require_finished:
        state = json.loads((root / 'system/final_state.json').read_text())
        if state['policy']['phase'] != 6 or not (root / 'system/sealed.json').exists():
            raise ValueError('run is not FINISHED and sealed')
        if 'events.jsonl' not in seen:
            raise ValueError('final manifest does not register event log')
    return {'run_id': manifest['run_id'], 'manifest_revision': manifest['manifest_revision'],
            'files_verified': len(seen), 'events_verified': len(events),
            'task1_frozen': manifest['task1'] is not None,
            'run_mode': manifest['run_mode']}


def export(directory, schema_directory, destination):
    result = audit(directory, schema_directory, require_finished=True)
    root = Path(directory).resolve()
    manifest = json.loads((root / 'manifest.json').read_text())
    destination = Path(destination).resolve()
    if destination.exists():
        raise ValueError('export destination already exists')
    if root == destination or root in destination.parents:
        raise ValueError('export must be outside the run directory')
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(destination, 'x:gz', dereference=True) as archive:
        for relative in ['manifest.json'] + sorted(item['path'] for item in manifest['files']):
            archive.add(root / relative, arcname=root.name + '/' + relative, recursive=False)
    result['export'] = str(destination)
    result['export_sha256'] = sha256(destination)
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory')
    parser.add_argument('--schemas', default='schemas')
    parser.add_argument('--finished', action='store_true')
    parser.add_argument('--export')
    options = parser.parse_args(args)
    result = (export(options.run_directory, options.schemas, options.export) if options.export else
              audit(options.run_directory, options.schemas, options.finished))
    print(json.dumps(result, ensure_ascii=False, indent=2))
