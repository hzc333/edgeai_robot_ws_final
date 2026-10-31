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

"""RGB-D and rigid-pose geometry helpers for the standalone P3 algorithms."""

from dataclasses import dataclass

import cv2
import numpy as np


def validate_camera_calibration(camera_matrix, distortion=None):
    matrix = np.array(camera_matrix, dtype=np.float64, copy=True)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('camera_matrix must be a finite 3x3 matrix')
    if (matrix[0, 0] <= 0 or matrix[1, 1] <= 0
            or not np.allclose(matrix[2], [0, 0, 1])
            or matrix[0, 1] != 0 or matrix[1, 0] != 0):
        raise ValueError('camera_matrix must be a standard positive-focal-length matrix')
    coefficients = np.array([] if distortion is None else distortion,
                            dtype=np.float64).reshape(-1)
    if (coefficients.size not in (0, 4, 5, 8, 12, 14)
            or not np.isfinite(coefficients).all()):
        raise ValueError('distortion must contain 0, 4, 5, 8, 12 or 14 finite coefficients')
    return matrix, coefficients


def depth_to_meters(depth_image, *, unit_scale_m=1.0):
    """Convert positive finite depth samples to metres; encode invalid samples as NaN."""
    if (isinstance(unit_scale_m, (bool, np.bool_))
            or not isinstance(unit_scale_m, (int, float, np.integer, np.floating))
            or not np.isfinite(unit_scale_m) or unit_scale_m <= 0):
        raise ValueError('unit_scale_m must be finite and positive')
    image = np.asarray(depth_image)
    if (image.ndim != 2 or image.size == 0
            or not (np.issubdtype(image.dtype, np.integer)
                    or np.issubdtype(image.dtype, np.floating))):
        raise ValueError('depth_image must be a nonempty 2D integer or floating-point array')
    meters = image.astype(np.float64) * float(unit_scale_m)
    meters[~np.isfinite(meters) | (meters <= 0)] = np.nan
    return meters


def back_project_pixels(pixels, depths_m, camera_matrix, distortion=None):
    """Back-project pixels in the optical frame, accounting for OpenCV lens distortion."""
    matrix, coefficients = validate_camera_calibration(camera_matrix, distortion)
    points = np.asarray(pixels, dtype=np.float64)
    if points.shape == (2,):
        points = points.reshape(1, 2)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('pixels must be one or more finite (u, v) pairs')
    depths = np.asarray(depths_m, dtype=np.float64)
    if depths.ndim == 0:
        depths = np.full(points.shape[0], float(depths))
    else:
        depths = depths.reshape(-1)
    if depths.size != points.shape[0] or not np.isfinite(depths).all() or np.any(depths <= 0):
        raise ValueError('depths_m must contain one finite positive depth per pixel')
    normalized = cv2.undistortPoints(
        points.reshape(-1, 1, 2), matrix,
        coefficients if coefficients.size else None).reshape(-1, 2)
    return np.column_stack((normalized * depths[:, None], depths))


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform mapping points from a source frame into a destination frame."""

    rotation: np.ndarray
    translation: np.ndarray

    def __post_init__(self):
        rotation = np.array(self.rotation, dtype=np.float64, copy=True)
        translation = np.array(self.translation, dtype=np.float64, copy=True).reshape(-1)
        if rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError('rotation and translation must have shapes (3, 3) and (3,)')
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise ValueError('rotation and translation must be finite')
        if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-7)
                or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-7)):
            raise ValueError('rotation must be a right-handed orthonormal matrix')
        rotation.setflags(write=False)
        translation.setflags(write=False)
        object.__setattr__(self, 'rotation', rotation)
        object.__setattr__(self, 'translation', translation)

    @classmethod
    def from_rvec_tvec(cls, rvec, tvec):
        rotation_vector = np.asarray(rvec, dtype=np.float64)
        translation = np.asarray(tvec, dtype=np.float64)
        if rotation_vector.size != 3:
            raise ValueError('rvec must contain three values')
        return cls(cv2.Rodrigues(rotation_vector.reshape(3, 1))[0], translation)

    @classmethod
    def from_matrix(cls, matrix):
        homogeneous = np.asarray(matrix, dtype=np.float64)
        if (homogeneous.shape != (4, 4) or not np.isfinite(homogeneous).all()
                or not np.allclose(homogeneous[3], [0, 0, 0, 1], atol=1e-7)):
            raise ValueError('homogeneous transform must be a finite 4x4 rigid matrix')
        return cls(homogeneous[:3, :3], homogeneous[:3, 3])

    def to_matrix(self):
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation
        return matrix

    def transform_points(self, points):
        values = np.asarray(points, dtype=np.float64)
        if values.shape == () or values.shape[-1] != 3 or not np.isfinite(values).all():
            raise ValueError('points must be finite with a final dimension of three')
        return values @ self.rotation.T + self.translation

    def inverse(self):
        rotation = self.rotation.T
        return RigidTransform(rotation, -rotation @ self.translation)

    def compose(self, other):
        """Return self @ other, applying other first and then self."""
        if not isinstance(other, RigidTransform):
            raise TypeError('other must be a RigidTransform')
        return RigidTransform(self.rotation @ other.rotation,
                              self.rotation @ other.translation + self.translation)


R_M_T = np.array([[0., 1., 0.],
                  [0., 0., 1.],
                  [1., 0., 0.]])
R_M_T.setflags(write=False)


def marker_to_target_pose(marker_pose):
    """Convert camera-from-M into camera-from-T per geometry contract v0.2."""
    if not isinstance(marker_pose, RigidTransform):
        raise TypeError('marker_pose must be a RigidTransform')
    return RigidTransform(marker_pose.rotation @ R_M_T, marker_pose.translation)
