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

"""ArUco observations in camera optical coordinates, per geometry contract v0.2."""

from dataclasses import dataclass
from typing import Mapping

import cv2
import numpy as np

from .geometry import depth_to_meters, validate_camera_calibration


@dataclass(frozen=True)
class TargetIdentity:
    """Configured target identity; deliberately independent of ROS enums."""

    target_id: str
    target_type: str


@dataclass(frozen=True)
class MarkerResult:
    """One detection. Unknown IDs have no identity or pose and are never valid."""

    marker_id: int
    corners: np.ndarray
    identity: TargetIdentity | None
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    reprojection_error_px: float | None = None
    rejection_reason: str | None = None
    depth_consistency_error_m: float | None = None
    ambiguity: 'PlanarPnpDiagnostics | None' = None

    @property
    def valid(self) -> bool:
        return (self.identity is not None and self.rvec is not None
                and self.rejection_reason is None)


@dataclass(frozen=True)
class PlanarPnpDiagnostics:
    """Pixel-only ambiguity measurements between positive-depth planar PnP candidates."""

    candidate_count: int
    reprojection_error_gap_px: float | None
    translation_separation_m: float | None
    rotation_separation_rad: float | None
    ambiguous: bool


class ArucoCore:
    """
    Detect and estimate M-to-camera pose, without TF or target-axis conversion.

    Images are uint8 grayscale or BGR (convert ROS RGB explicitly in a future
    adapter). Camera calibration must correspond to the supplied image size.
    Translation is in metres; rotation is an OpenCV Rodrigues vector in radians.
    """

    def __init__(self, dictionary: str, marker_size_m: float,
                 id_mapping: Mapping, *, max_reprojection_error_px: float | None = None,
                 ambiguity_error_margin_px: float = 1.0):
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, 'ArucoDetector'):
            raise RuntimeError('OpenCV ArucoDetector is required; use project OpenCV 4.12')
        if not isinstance(dictionary, str) or not dictionary.startswith('DICT_'):
            raise ValueError('dictionary must name a predefined OpenCV DICT_* dictionary')
        dictionary_id = getattr(cv2.aruco, dictionary, None)
        if not isinstance(dictionary_id, int):
            raise ValueError(f'unknown ArUco dictionary: {dictionary}')
        if (isinstance(marker_size_m, bool)
                or not isinstance(marker_size_m, (int, float, np.integer, np.floating))
                or not np.isfinite(marker_size_m) or marker_size_m <= 0):
            raise ValueError('marker_size_m must be finite and positive')
        if max_reprojection_error_px is not None and (
                isinstance(max_reprojection_error_px, bool)
                or not isinstance(max_reprojection_error_px,
                                  (int, float, np.integer, np.floating))
                or not np.isfinite(max_reprojection_error_px) or max_reprojection_error_px <= 0):
            raise ValueError('max_reprojection_error_px must be finite and positive')
        if (isinstance(ambiguity_error_margin_px, bool)
                or not isinstance(ambiguity_error_margin_px,
                                  (int, float, np.integer, np.floating))
                or not np.isfinite(ambiguity_error_margin_px)
                or ambiguity_error_margin_px < 0):
            raise ValueError('ambiguity_error_margin_px must be finite and nonnegative')
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.marker_size_m = float(marker_size_m)
        self.max_reprojection_error_px = max_reprojection_error_px
        self.ambiguity_error_margin_px = float(ambiguity_error_margin_px)
        self.id_mapping = {}
        if not isinstance(id_mapping, Mapping) or not id_mapping:
            raise ValueError('id_mapping must be a nonempty mapping')
        for marker_id, entry in id_mapping.items():
            if isinstance(marker_id, bool) or not isinstance(marker_id, (int, np.integer)):
                raise ValueError('marker IDs must be integers')
            if not 0 <= marker_id < len(self.dictionary.bytesList):
                raise ValueError(f'marker ID outside dictionary: {marker_id}')
            if not isinstance(entry, Mapping):
                raise ValueError('ID mapping entries must be mappings')
            target_id, target_type = entry.get('target_id'), entry.get('type')
            if not isinstance(target_id, str) or not target_id.strip():
                raise ValueError('target_id must be a nonempty string')
            if target_type not in ('normal', 'cooperative'):
                raise ValueError('target type must be normal or cooperative')
            self.id_mapping[int(marker_id)] = TargetIdentity(target_id, target_type)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, parameters)

    @classmethod
    def from_resolved_parameters(cls, parameters, **kwargs):
        """Consume the existing common resolver result; do not duplicate fallback rules."""
        return cls(parameters.require('aruco.dictionary'),
                   parameters.require('aruco.marker_size_m'),
                   parameters.require('aruco.id_mapping'), **kwargs)

    @classmethod
    def from_repository_config(cls, run_mode='development', config_dir=None, **kwargs):
        """
        Load competition_parameters.yaml with the ROS-free edgeai_common resolver.

        Competition mode intentionally preserves its existing missing-official-
        parameter gate, including requirements outside perception.
        """
        from edgeai_common.config import resolve_parameters
        return cls.from_resolved_parameters(resolve_parameters(run_mode, config_dir), **kwargs)

    def object_points(self) -> np.ndarray:
        """TL, TR, BR, BL, black-border outer edge; x right, y up, z to viewer."""
        half = self.marker_size_m / 2
        return np.array([[-half, half, 0], [half, half, 0],
                         [half, -half, 0], [-half, -half, 0]], dtype=np.float64)

    @staticmethod
    def _calibration(camera_matrix, distortion):
        return validate_camera_calibration(camera_matrix, distortion)

    @staticmethod
    def _depth_error(objects, rotation, translation, matrix, coefficients, depth_image_m):
        half = float(np.max(np.abs(objects[:, :2]))) / 3
        coordinates = np.array([(x, y, 0.) for y in (-half, 0., half)
                                for x in (-half, 0., half)])
        pixels = cv2.projectPoints(coordinates, rotation, translation, matrix,
                                   coefficients)[0].reshape(-1, 2)
        camera_points = coordinates @ cv2.Rodrigues(rotation)[0].T + translation
        image_height, image_width = depth_image_m.shape
        errors = []
        for (u, v), expected in zip(pixels, camera_points[:, 2]):
            column, row = np.rint([u, v]).astype(int)
            left, right = max(0, column - 1), min(image_width, column + 2)
            top, bottom = max(0, row - 1), min(image_height, row + 2)
            if left >= right or top >= bottom:
                continue
            samples = depth_image_m[top:bottom, left:right]
            samples = samples[np.isfinite(samples) & (samples > 0)]
            if samples.size:
                errors.append(abs(float(np.median(samples)) - float(expected)))
        return float(np.median(errors)) if len(errors) >= 3 else None

    @staticmethod
    def _ambiguity_diagnostics(candidates, error_margin_px):
        if len(candidates) < 2:
            return PlanarPnpDiagnostics(len(candidates), None, None, None, False)
        ranked = sorted(candidates, key=lambda candidate: candidate[0])
        best, second = ranked[:2]
        error_gap = max(0.0, second[0] - best[0])
        rotation_delta = cv2.Rodrigues(second[1])[0] @ cv2.Rodrigues(best[1])[0].T
        angle = float(np.arccos(np.clip((np.trace(rotation_delta) - 1) / 2, -1, 1)))
        return PlanarPnpDiagnostics(
            len(candidates), error_gap,
            float(np.linalg.norm(second[2] - best[2])), angle,
            error_gap <= error_margin_px)

    def estimate_pose(self, marker_id, corners, camera_matrix, distortion=None, *,
                      depth_image=None, depth_scale_m=None,
                      depth_tolerance_m=0.05) -> MarkerResult:
        """
        Solve both IPPE candidates; retain the positive-depth, lowest-RMS solution.

        RMS is sqrt(mean(sum((observed - projected)**2, axis=1))) in pixels.
        Unknown IDs are returned as rejected diagnostics without running PnP.
        """
        matrix, coefficients = self._calibration(camera_matrix, distortion)
        depth_m = None
        if depth_image is not None:
            if depth_scale_m is None:
                raise ValueError('depth_scale_m must be specified when depth_image is provided')
            depth_m = depth_to_meters(depth_image, unit_scale_m=depth_scale_m)
            if (isinstance(depth_tolerance_m, bool)
                    or not isinstance(depth_tolerance_m,
                                      (int, float, np.integer, np.floating))
                    or not np.isfinite(depth_tolerance_m) or depth_tolerance_m <= 0):
                raise ValueError('depth_tolerance_m must be finite and positive')
        points = np.array(corners, dtype=np.float64, copy=True)
        if points.shape not in ((4, 2), (1, 4, 2)) or not np.isfinite(points).all():
            raise ValueError('corners must contain four finite image points in TL/TR/BR/BL order')
        points = points.reshape(4, 2)
        if isinstance(marker_id, bool) or not isinstance(marker_id, (int, np.integer)):
            raise ValueError('marker_id must be an integer')
        identity = self.id_mapping.get(marker_id)

        def rejected(reason, ambiguity=None):
            return MarkerResult(int(marker_id), points, identity, rejection_reason=reason,
                                ambiguity=ambiguity)

        if identity is None:
            return rejected('unknown_marker_id')
        if not cv2.isContourConvex(points.astype(np.float32)) or abs(
                cv2.contourArea(points.astype(np.float32))) < 1e-6:
            return rejected('degenerate_corners')
        objects = self.object_points()
        try:
            solved, rotations, translations, *_ = cv2.solvePnPGeneric(
                objects, points, matrix, coefficients, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return rejected('pose_estimation_failed')
        if not solved:
            return rejected('pose_estimation_failed')
        candidates = []
        for rotation, translation in zip(rotations, translations):
            rotation, translation = rotation.reshape(3), translation.reshape(3)
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                continue
            camera_points = objects @ cv2.Rodrigues(rotation)[0].T + translation
            if np.any(camera_points[:, 2] <= 0):
                continue
            projected, _ = cv2.projectPoints(objects, rotation, translation, matrix, coefficients)
            residuals = points - projected.reshape(4, 2)
            error = float(np.sqrt(np.mean(np.sum(residuals ** 2, axis=1))))
            if np.isfinite(error):
                depth_error = (None if depth_m is None else self._depth_error(
                    objects, rotation, translation, matrix, coefficients, depth_m))
                candidates.append((error, rotation.copy(), translation.copy(), depth_error))
        ambiguity = self._ambiguity_diagnostics(candidates, self.ambiguity_error_margin_px)
        if not candidates:
            return rejected('no_positive_depth_solution', ambiguity)
        selected_candidates = candidates
        if depth_m is not None:
            sampled = [candidate for candidate in candidates if candidate[3] is not None]
            if not sampled:
                return rejected('depth_unavailable', ambiguity)
            selected_candidates = [
                candidate for candidate in sampled if candidate[3] <= depth_tolerance_m]
            if not selected_candidates:
                return rejected('depth_inconsistent', ambiguity)
        error, rotation, translation, depth_error = min(
            selected_candidates, key=lambda candidate: candidate[0])
        reason = None
        if self.max_reprojection_error_px is not None and error > self.max_reprojection_error_px:
            reason = 'reprojection_error_exceeded'
        return MarkerResult(int(marker_id), points, identity, rotation, translation, error, reason,
                            depth_error, ambiguity)

    def detect(self, image, camera_matrix, distortion=None, *, depth_image=None,
               depth_scale_m=None, depth_tolerance_m=0.05) -> list[MarkerResult]:
        """Return known detections and explicit rejected diagnostics, in marker-ID order."""
        matrix, coefficients = self._calibration(camera_matrix, distortion)
        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.size == 0:
            raise ValueError('image must be a nonempty uint8 array')
        if image.ndim == 3 and image.shape[2] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        elif image.ndim != 2:
            raise ValueError('image must be grayscale or three-channel BGR')
        if depth_image is not None:
            if depth_scale_m is None:
                raise ValueError('depth_scale_m must be specified when depth_image is provided')
            depth_m = depth_to_meters(depth_image, unit_scale_m=depth_scale_m)
            if depth_m.shape != image.shape:
                raise ValueError('depth_image must be registered and match the image dimensions')
            if (isinstance(depth_tolerance_m, bool)
                    or not isinstance(depth_tolerance_m,
                                      (int, float, np.integer, np.floating))
                    or not np.isfinite(depth_tolerance_m) or depth_tolerance_m <= 0):
                raise ValueError('depth_tolerance_m must be finite and positive')
            depth_image = depth_m
            depth_scale_m = 1.0
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None:
            return []
        return sorted((self.estimate_pose(
            int(marker_id), points, matrix, coefficients, depth_image=depth_image,
            depth_scale_m=depth_scale_m, depth_tolerance_m=depth_tolerance_m)
                       for marker_id, points in zip(ids.reshape(-1), corners)),
                      key=lambda result: result.marker_id)
