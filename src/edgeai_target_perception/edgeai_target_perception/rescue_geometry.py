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

"""Versioned rescue-zone geometry shared by fusion output and Task 1 export."""

import math

import numpy as np

from .geometry import marker_to_target_pose, RigidTransform


def normalize_angle(angle):
    """Normalize a finite angle to [-pi, pi)."""
    if not math.isfinite(angle):
        raise ValueError('angle must be finite')
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def target_planar_pose(marker_pose):
    """Project a vertical marker pose to the contract's map-plane target pose."""
    target_pose = marker_to_target_pose(marker_pose)
    forward = target_pose.rotation[:, 0]
    if abs(float(forward[2])) > math.sqrt(0.5):
        raise ValueError('marker_not_vertical')
    yaw = normalize_angle(math.atan2(float(forward[1]), float(forward[0])))
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cosine, -sine, 0.],
        [sine, cosine, 0.],
        [0., 0., 1.],
    ])
    return RigidTransform(rotation, target_pose.translation), yaw


def validate_development_geometry(geometry, expected_version):
    """Validate the configured version and offsets needed by development geometry."""
    if not isinstance(geometry, dict):
        raise ValueError('rescue_geometry must be a mapping')
    if geometry.get('version') != expected_version or not expected_version:
        raise ValueError('development rescue geometry version mismatch')
    if geometry.get('zone_orientation') != 'face_marker':
        raise ValueError('unsupported rescue-zone orientation')
    for target_type in ('normal', 'cooperative'):
        rule = geometry.get(target_type)
        if not isinstance(rule, dict):
            raise ValueError(f'{target_type} rescue geometry must be a mapping')
        forward = rule.get('forward_offset_m')
        if (isinstance(forward, bool) or not isinstance(forward, (int, float))
                or not math.isfinite(forward) or forward < 0):
            raise ValueError(f'{target_type}.forward_offset_m must be finite and nonnegative')
    normal = geometry['normal']
    cooperative = geometry['cooperative']
    if 'lateral_offsets_m' in normal:
        raise ValueError('normal rescue geometry must not define lateral offsets')
    lateral = cooperative.get('lateral_offsets_m')
    if (not isinstance(lateral, list) or len(lateral) != 2
            or any(isinstance(item, bool) or not isinstance(item, (int, float))
                   or not math.isfinite(item) for item in lateral)):
        raise ValueError('cooperative.lateral_offsets_m must contain two finite offsets')
    if lateral[0] == lateral[1]:
        raise ValueError('cooperative lateral offsets must be distinct')


def rescue_zone_poses(target_pose, yaw, target_type, geometry):
    """Return indexed map-frame rescue poses using the configured geometry version."""
    if not isinstance(target_pose, RigidTransform):
        raise TypeError('target_pose must be a RigidTransform')
    if target_type not in ('normal', 'cooperative'):
        raise ValueError('target_type must be normal or cooperative')
    rule = geometry[target_type]
    forward_offset = float(rule['forward_offset_m'])
    lateral_offsets = [0.] if target_type == 'normal' else rule['lateral_offsets_m']
    forward = np.array([math.cos(yaw), math.sin(yaw)])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    zone_yaw = normalize_angle(yaw + math.pi)
    return tuple(
        (
            index,
            float(target_pose.translation[0]
                  + forward_offset * forward[0] + lateral * left[0]),
            float(target_pose.translation[1]
                  + forward_offset * forward[1] + lateral * left[1]),
            0.,
            zone_yaw,
        )
        for index, lateral in enumerate(lateral_offsets)
    )
