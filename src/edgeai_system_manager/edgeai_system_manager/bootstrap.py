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

"""Create reproducible run metadata without changing the locked dependencies."""

import json
import os
from pathlib import Path
import subprocess
import tarfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from edgeai_common.config import (
    find_config_dir, load_yaml, MissingOfficialParameters, resolve_parameters, ResolvedParameters)
import yaml

from .storage import atomic_bytes, Schemas, sha256, utc_now


def resolve_for_manager(mode, config):
    try:
        return resolve_parameters(mode, config)
    except MissingOfficialParameters as error:
        if mode != 'competition':
            raise
        # A competition manager must remain observable in PREPARATION while
        # reporting missing official fields; motion remains strictly forbidden.
        document = load_yaml(Path(config) / 'competition_parameters.yaml')
        return ResolvedParameters(mode, document['official'], [], error.missing)


def git_identity(root):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()
    return {'commit': git('rev-parse', 'HEAD'), 'branch': git('branch', '--show-current'),
            'dirty': bool(git('status', '--porcelain'))}


def initialize_artifacts(store, config_dir, mode, robots):
    config = find_config_dir(config_dir)
    root = config.parent
    schemas = Schemas(root / 'schemas')
    resolved = resolve_for_manager(mode, config)
    parameters = {'run_mode': mode, 'values': resolved.values, 'fallbacks': resolved.fallbacks,
                  'missing_for_motion': resolved.missing_for_motion}
    data = yaml.safe_dump(parameters, sort_keys=True).encode()
    parameters_path = store.path('system/parameters.yaml')
    if parameters_path.exists():
        if parameters_path.read_bytes() != data:
            raise ValueError('resolved parameters changed during this run')
    else:
        atomic_bytes(parameters_path, data)
    resolved.sha256 = sha256(parameters_path)
    source_paths = subprocess.check_output(
        ['git', '-C', str(root), 'ls-files', '--cached', '--others', '--exclude-standard', '-z']
    ).decode().split('\0')
    source_hashes = {name: sha256(root / name) for name in sorted(set(source_paths))
                     if name and (root / name).is_file() and not (root / name).is_symlink()}
    source_path = store.path('system/source_hashes.json')
    if source_path.exists():
        if json.loads(source_path.read_text()) != source_hashes:
            raise ValueError('source tree changed during run; create an explicit new run')
    else:
        store.write('system/source_hashes.json', source_hashes)
        with tarfile.open(store.path('system/source_snapshot.tar.gz'), 'x:gz') as archive:
            for name in source_hashes:
                archive.add(root / name, arcname=name, recursive=False)
    versions = store.read('system/component_versions.json')
    if versions is None:
        packages = []
        for path in sorted((root / 'src').glob('*/package.xml')):
            xml = ET.parse(path).getroot()
            packages.append({'name': xml.findtext('name'), 'version': xml.findtext('version'),
                             'origin': 'workspace'})
        for name in ('slam_toolbox', 'nav2_bringup', 'ros_gz_sim'):
            xml = ET.parse(Path(get_package_share_directory(name)) / 'package.xml').getroot()
            packages.append({'name': name, 'version': xml.findtext('version'),
                             'origin': 'third_party'})
        import cv2
        packages.append({'name': 'opencv', 'version': cv2.__version__, 'origin': 'third_party'})
        versions = {'schema_version': 1, 'run_id': store.run_id, 'generated_at': utc_now(),
                    'environment': 'pixi', 'ros_distro': 'jazzy',
                    'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', ''),
                    'lockfile_sha256': sha256(root / 'pixi.lock'), 'git': git_identity(root),
                    'packages': packages, 'models': []}
        schemas.check('component_versions', versions)
        store.write('system/component_versions.json', versions)
    else:
        schemas.check('component_versions', versions)
        if versions['lockfile_sha256'] != sha256(root / 'pixi.lock'):
            raise ValueError('dependency lock changed during run')
        if versions['git']['commit'] != git_identity(root)['commit']:
            raise ValueError('Git revision changed during run')
    manifest = store.read('manifest.json')
    if manifest is None:
        manifest = {'schema_version': 1, 'run_id': store.run_id, 'manifest_revision': 1,
                    'updated_at': utc_now(), 'run_mode': mode,
                    'rulebook_version': load_yaml(config / 'competition_parameters.yaml')[
                        'rulebook']['version'],
                    'git': {key: versions['git'][key] for key in ('commit', 'dirty')},
                    'robots': robots, 'task1': None, 'resets': 0, 'files': []}
        for relative, role in [('system/parameters.yaml', 'parameters'),
                               ('system/component_versions.json', 'component_versions'),
                               ('system/source_hashes.json', 'other'),
                               ('system/source_snapshot.tar.gz', 'other')]:
            path = store.path(relative)
            manifest['files'].append({'path': relative, 'role': role, 'stage': 'system',
                                      'sha256': sha256(path), 'size_bytes': path.stat().st_size})
        schemas.check('manifest', manifest)
        store.write('manifest.json', manifest)
    return resolved, schemas


def targets_to_document(message, geometry_version, geometry_source):
    def pose(value):
        return {'position': {key: getattr(value.position, key) for key in ('x', 'y', 'z')},
                'orientation': {key: getattr(value.orientation, key)
                                for key in ('x', 'y', 'z', 'w')}}
    import math
    targets = []
    for item in message.targets:
        if (item.run_id != message.run_id or item.map_id != message.map_id
                or item.map_revision != message.map_revision or item.header.frame_id != 'map'):
            raise ValueError('target identity mismatch')
        if item.target_type not in (1, 2):
            raise ValueError('unknown target type cannot be frozen')
        q = item.marker_pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
        if any(zone.header.frame_id != 'map' for zone in item.rescue_poses):
            raise ValueError('rescue pose frame mismatch')
        targets.append({'target_id': item.target_id, 'marker_id': item.marker_id,
                        'target_type': 'normal' if item.target_type == 1 else 'cooperative',
                        'marker_pose': pose(item.marker_pose.pose), 'marker_yaw_rad': yaw,
                        'covariance': list(item.marker_pose.covariance),
                        'rescue_zones': [{'index': i, 'pose': pose(zone.pose)}
                                         for i, zone in enumerate(item.rescue_poses)],
                        'observation_count': item.observation_count, 'confidence': item.confidence,
                        'observed_by': list(item.observed_by),
                        'eligible_for_basic_rescue': item.eligible_for_basic_rescue})
    document = {'schema_version': 2, 'run_id': message.run_id, 'revision': message.revision,
                'frozen_at': utc_now(), 'frame_id': 'map',
                'map': {'map_id': message.map_id, 'revision': message.map_revision},
                'rescue_geometry': {'version': geometry_version, 'source': geometry_source},
                'targets': targets}
    json.dumps(document, allow_nan=False)
    return document
