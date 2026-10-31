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

"""ROS-independent target association, de-duplication, and fusion."""

from dataclasses import dataclass

import numpy as np

from .geometry import RigidTransform


@dataclass(frozen=True)
class FusionObservation:
    """A single map-frame marker pose and its ROS timestamp."""

    stamp_ns: int
    robot_id: str
    marker_id: int
    target_id: str
    target_type: str
    position: np.ndarray
    rotation: np.ndarray
    covariance: np.ndarray
    confidence: float

    def __post_init__(self):
        """Validate and freeze array-valued observation fields."""
        transform = RigidTransform(self.rotation, self.position)
        covariance = np.array(self.covariance, dtype=np.float64, copy=True)
        if covariance.shape != (6, 6) or not np.isfinite(covariance).all():
            raise ValueError('covariance must be a finite 6x6 matrix')
        if not np.allclose(covariance, covariance.T, atol=1e-8):
            raise ValueError('covariance must be symmetric')
        if np.linalg.eigvalsh(covariance).min() < -1e-7:
            raise ValueError('covariance must be positive semidefinite')
        if not isinstance(self.stamp_ns, int) or self.stamp_ns < 0:
            raise ValueError('stamp_ns must be a nonnegative integer')
        if not isinstance(self.robot_id, str) or self.robot_id not in {
                f'robot_{index}' for index in range(1, 5)}:
            raise ValueError('robot_id must be robot_1 through robot_4')
        if isinstance(self.marker_id, bool) or not isinstance(self.marker_id, int):
            raise ValueError('marker_id must be an integer')
        if not isinstance(self.target_id, str) or not self.target_id:
            raise ValueError('target_id must be a nonempty string')
        if self.target_type not in ('normal', 'cooperative'):
            raise ValueError('target_type must be normal or cooperative')
        if (isinstance(self.confidence, bool)
                or not isinstance(self.confidence, (int, float, np.number))
                or not np.isfinite(self.confidence) or not 0 <= self.confidence <= 1):
            raise ValueError('confidence must be finite and in [0, 1]')
        position = np.array(transform.translation, copy=True)
        rotation = np.array(transform.rotation, copy=True)
        covariance.setflags(write=False)
        position.setflags(write=False)
        rotation.setflags(write=False)
        object.__setattr__(self, 'position', position)
        object.__setattr__(self, 'rotation', rotation)
        object.__setattr__(self, 'covariance', covariance)
        object.__setattr__(self, 'confidence', float(self.confidence))


@dataclass(frozen=True)
class FusedTarget:
    """A conservative estimate based on at most one current sample per robot."""

    target_id: str
    marker_id: int
    target_type: str
    stamp_ns: int
    position: np.ndarray
    rotation: np.ndarray
    covariance: np.ndarray
    confidence: float
    observation_count: int
    observed_by: tuple[str, ...]


def transform_observation(observation, transform):
    """Apply a source-to-map rigid transform and rotate its pose covariance."""
    if not isinstance(observation, FusionObservation):
        raise TypeError('observation must be a FusionObservation')
    if not isinstance(transform, RigidTransform):
        raise TypeError('transform must be a RigidTransform')
    rotation = transform.rotation @ observation.rotation
    position = transform.transform_points(observation.position)
    jacobian = np.zeros((6, 6), dtype=np.float64)
    jacobian[:3, :3] = transform.rotation
    jacobian[3:, 3:] = transform.rotation
    covariance = jacobian @ observation.covariance @ jacobian.T
    return FusionObservation(
        stamp_ns=observation.stamp_ns,
        robot_id=observation.robot_id,
        marker_id=observation.marker_id,
        target_id=observation.target_id,
        target_type=observation.target_type,
        position=position,
        rotation=rotation,
        covariance=covariance,
        confidence=observation.confidence,
    )


def _average_rotation(rotations, weights):
    matrix = sum(weight * rotation for rotation, weight in zip(rotations, weights))
    left, _, right = np.linalg.svd(matrix)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(left @ right)
    return left @ correction @ right


def _rotation_residual(reference, rotation):
    """Return the small-angle vector from a reference rotation to a sample."""
    relative = reference.T @ rotation
    cosine = float(np.clip((np.trace(relative) - 1) / 2, -1, 1))
    angle = float(np.arccos(cosine))
    skew = np.array([relative[2, 1] - relative[1, 2],
                     relative[0, 2] - relative[2, 0],
                     relative[1, 0] - relative[0, 1]])
    if angle < 1e-8:
        return skew / 2
    if np.pi - angle < 1e-5:
        symmetric = (relative + np.eye(3)) / 2
        _, vectors = np.linalg.eigh(symmetric)
        axis = vectors[:, -1]
        if np.linalg.norm(skew) > 1e-8 and np.dot(axis, skew) < 0:
            axis = -axis
        return angle * axis
    return angle * skew / (2 * np.sin(angle))


class TargetRegistry:
    """Versioned target registry with known-identity association and source replacement."""

    def __init__(self, id_mapping, *, minimum_confidence=0.6,
                 position_gate_m=0.5, stale_after_ns=2_000_000_000,
                 covariance_floor=1e-4):
        """Create a registry using the resolved marker identity table and safety gates."""
        if not isinstance(id_mapping, dict) or not id_mapping:
            raise ValueError('id_mapping must be a nonempty dictionary')
        self.definitions = {}
        target_ids = set()
        for marker_id, entry in id_mapping.items():
            if isinstance(marker_id, bool) or not isinstance(marker_id, int):
                raise ValueError('configured marker IDs must be integers')
            if not isinstance(entry, dict):
                raise ValueError('configured marker definitions must be dictionaries')
            target_id, target_type = entry.get('target_id'), entry.get('type')
            if not isinstance(target_id, str) or not target_id:
                raise ValueError('configured target IDs must be nonempty strings')
            if target_type not in ('normal', 'cooperative'):
                raise ValueError('configured target type must be normal or cooperative')
            if target_id in target_ids:
                raise ValueError(f'target_id {target_id!r} must identify one marker ID')
            target_ids.add(target_id)
            self.definitions[marker_id] = (target_id, target_type)
        if (not np.isfinite(minimum_confidence)
                or not 0 <= minimum_confidence <= 1):
            raise ValueError('minimum_confidence must be finite and in [0, 1]')
        if not np.isfinite(position_gate_m) or position_gate_m <= 0:
            raise ValueError('position_gate_m must be finite and positive')
        if not isinstance(stale_after_ns, int) or stale_after_ns <= 0:
            raise ValueError('stale_after_ns must be a positive integer')
        if not np.isfinite(covariance_floor) or covariance_floor <= 0:
            raise ValueError('covariance_floor must be finite and positive')
        self.minimum_confidence = float(minimum_confidence)
        self.position_gate_m = float(position_gate_m)
        self.stale_after_ns = stale_after_ns
        self.covariance_floor = float(covariance_floor)
        self._sources = {}
        self.revision = 0

    def reset(self):
        """Start a new run/map registry version."""
        self._sources.clear()
        self.revision = 0

    def ingest(self, observation, now_ns):
        """Accept a fresh consistent observation, replacing that robot's prior sample."""
        if not isinstance(observation, FusionObservation):
            raise TypeError('observation must be a FusionObservation')
        if not isinstance(now_ns, int) or now_ns < 0:
            raise ValueError('now_ns must be a nonnegative integer')
        identity = self.definitions.get(observation.marker_id)
        if identity != (observation.target_id, observation.target_type):
            return False
        if (observation.confidence <= 0
                or observation.confidence < self.minimum_confidence):
            return False
        if now_ns - observation.stamp_ns > self.stale_after_ns:
            return False
        sources = self._sources.get(observation.target_id, {})
        previous = sources.get(observation.robot_id)
        if previous is not None and observation.stamp_ns <= previous.stamp_ns:
            return False

        references = [item.position for robot_id, item in sources.items()
                      if robot_id != observation.robot_id]
        if references:
            center = np.median(np.stack(references), axis=0)
            if np.linalg.norm(observation.position - center) > self.position_gate_m:
                return False
        sources[observation.robot_id] = observation
        self._sources[observation.target_id] = sources
        self.revision += 1
        return True

    def prune(self, now_ns):
        """Drop stale robot samples and remove registry entries with no live evidence."""
        changed = False
        for target_id in tuple(self._sources):
            sources = self._sources[target_id]
            for robot_id, observation in tuple(sources.items()):
                if now_ns - observation.stamp_ns > self.stale_after_ns:
                    del sources[robot_id]
                    changed = True
            if not sources:
                del self._sources[target_id]
        if changed:
            self.revision += 1
        return changed

    def snapshot(self):
        """Return deterministically ordered estimates with one contribution per robot."""
        estimates = []
        for target_id in sorted(self._sources):
            sources = self._sources[target_id]
            items = list(sources.values())
            position_covariances = [item.covariance[:3, :3] for item in items]
            variances = [max(float(np.trace(value) / 3), self.covariance_floor)
                         for value in position_covariances]
            raw_weights = np.array([
                item.confidence / variance for item, variance in zip(items, variances)
            ], dtype=np.float64)
            weights = raw_weights / raw_weights.sum()
            positions = np.stack([item.position for item in items])
            position = np.average(positions, axis=0, weights=weights)
            rotation = _average_rotation([item.rotation for item in items], weights)
            covariance = sum(
                weight * item.covariance for item, weight in zip(items, weights))
            residuals = positions - position
            covariance[:3, :3] += sum(
                weight * np.outer(residual, residual)
                for residual, weight in zip(residuals, weights))
            rotation_residuals = [
                _rotation_residual(rotation, item.rotation) for item in items]
            covariance[3:, 3:] += sum(
                weight * np.outer(residual, residual)
                for residual, weight in zip(rotation_residuals, weights))
            covariance = (covariance + covariance.T) / 2
            confidence = 1.0 - float(np.prod([1.0 - item.confidence for item in items]))
            definition = self.definitions[items[0].marker_id]
            estimates.append(FusedTarget(
                target_id=target_id,
                marker_id=items[0].marker_id,
                target_type=definition[1],
                stamp_ns=max(item.stamp_ns for item in items),
                position=position,
                rotation=rotation,
                covariance=covariance,
                confidence=min(confidence, 1.0),
                observation_count=len(items),
                observed_by=tuple(sorted(sources)),
            ))
        return tuple(estimates)
