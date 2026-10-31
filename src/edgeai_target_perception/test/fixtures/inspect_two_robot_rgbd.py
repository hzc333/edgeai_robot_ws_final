#!/usr/bin/env python3
"""Log ArUco corners, PnP probes, and RGB/depth edge alignment in Gazebo."""

import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'src' / 'edgeai_common'))

import cv2  # noqa: E402
from cv_bridge import CvBridge  # noqa: E402
from edgeai_common.config import load_runtime_contract, resolve_parameters  # noqa: E402
from edgeai_common.qos import qos_from_contract  # noqa: E402
from edgeai_target_perception.aruco_core import ArucoCore  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

ROBOT_IDS = ('robot_1', 'robot_2')
CAMERA_ORIGIN = {
    'robot_1': np.array([-1.4125, -1.5, 0.1]),
    'robot_2': np.array([-1.4125, -0.8, 0.1]),
}
MARKER_CENTER = np.array([-0.45, -1.15, 0.15])


def _rotation_x(angle):
    """Return the right-handed X-axis rotation matrix."""
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[1., 0., 0.], [0., cosine, -sine], [0., sine, cosine]])


def _rotation_y(angle):
    """Return the right-handed Y-axis rotation matrix."""
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, 0., sine], [0., 1., 0.], [-sine, 0., cosine]])


def _rotation_z(angle):
    """Return the right-handed Z-axis rotation matrix."""
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine, 0.], [sine, cosine, 0.], [0., 0., 1.]])


class GeometryProbe(Node):
    """Capture one synchronized RGB-D frame per robot and print measured geometry."""

    def __init__(self):
        """Subscribe to both camera streams and prepare configured ArUco solvers."""
        super().__init__('two_robot_rgbd_geometry_probe')
        qos = qos_from_contract(load_runtime_contract(), 'sensor_data')
        self.bridge = CvBridge()
        self.camera_info = {}
        self.depth_images = {}
        self.completed = set()
        self.cores = {
            robot_id: ArucoCore.from_resolved_parameters(
                resolve_parameters('development', str(ROOT / 'config')))
            for robot_id in ROBOT_IDS
        }
        for robot_id in ROBOT_IDS:
            self.create_subscription(
                CameraInfo, f'/{robot_id}/camera/camera_info',
                lambda message, expected=robot_id: self._on_camera_info(message, expected),
                qos,
            )
            self.create_subscription(
                Image, f'/{robot_id}/camera/depth_image',
                lambda message, expected=robot_id: self._on_depth(message, expected),
                qos,
            )
            self.create_subscription(
                Image, f'/{robot_id}/camera/image',
                lambda message, expected=robot_id: self._on_rgb(message, expected),
                qos,
            )

    def _on_camera_info(self, message, robot_id):
        self.camera_info[robot_id] = message

    def _on_depth(self, message, robot_id):
        self.depth_images[robot_id] = message

    def _on_rgb(self, message, robot_id):
        if (robot_id in self.completed or robot_id not in self.camera_info
                or robot_id not in self.depth_images):
            return
        depth_message = self.depth_images[robot_id]
        info = self.camera_info[robot_id]
        rgb_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        depth_stamp = (depth_message.header.stamp.sec
                       + depth_message.header.stamp.nanosec * 1e-9)
        stamp_delta_s = abs(rgb_stamp - depth_stamp)
        if stamp_delta_s > 0.05:
            return

        rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough')
        matrix = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(info.d, dtype=np.float64)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        corners_list, ids, _ = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters()).detectMarkers(rgb)
        if ids is None or 1 not in ids.reshape(-1):
            return
        corners = corners_list[ids.reshape(-1).tolist().index(1)].reshape(4, 2)
        self._report(robot_id, message, depth_message, info, rgb, depth, matrix,
                     distortion, corners)
        self.completed.add(robot_id)

    def _report(self, robot_id, rgb_message, depth_message, info, rgb, depth, matrix,
                distortion, corners):
        rgb_stamp = (rgb_message.header.stamp.sec
                     + rgb_message.header.stamp.nanosec * 1e-9)
        depth_stamp = (depth_message.header.stamp.sec
                       + depth_message.header.stamp.nanosec * 1e-9)
        stamp_delta_s = abs(rgb_stamp - depth_stamp)
        core = self.cores[robot_id]
        result = core.estimate_pose(1, corners, matrix, distortion)
        print(
            f'\n{robot_id}: corners TL/TR/BR/BL px={np.round(corners, 2).tolist()}',
            flush=True,
        )
        print(
            f'{robot_id}: frame rgb/depth={rgb_message.header.frame_id}/'
            f'{depth_message.header.frame_id}; encoding={rgb_message.encoding}/'
            f'{depth_message.encoding}; size={rgb.shape[1]}x{rgb.shape[0]}/'
            f'{depth.shape[1]}x{depth.shape[0]}; stamp_delta_s={stamp_delta_s:.9f}',
            flush=True,
        )
        print(
            f'{robot_id}: CameraInfo K={np.round(matrix, 5).tolist()} '
            f'D={np.round(distortion, 6).tolist()} '
            f'PnP_tvec_m={np.round(result.tvec, 5).tolist()} '
            f'PnP_RMS_px={result.reprojection_error_px:.4f} '
            f'configured_marker_side_m={core.marker_size_m:.3f}',
            flush=True,
        )

        sample_half = float(np.max(np.abs(core.object_points()[:, :2]))) / 3
        local_samples = np.array([
            (x, y, 0.)
            for y in (-sample_half, 0., sample_half)
            for x in (-sample_half, 0., sample_half)
        ])
        _, rotations, translations, _ = cv2.solvePnPGeneric(
            core.object_points(), corners, matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        for candidate_index, (rotation, translation) in enumerate(zip(rotations, translations)):
            pixels = cv2.projectPoints(
                local_samples, rotation, translation, matrix, distortion)[0].reshape(-1, 2)
            camera_points = (
                local_samples @ cv2.Rodrigues(rotation)[0].T + translation.reshape(3))
            measurements = []
            for pixel, point in zip(pixels, camera_points):
                column, row = np.rint(pixel).astype(int)
                patch = depth[row - 1:row + 2, column - 1:column + 2].astype(np.float64)
                measurements.append({
                    'px': np.round(pixel, 2).tolist(),
                    'pred_z_m': round(float(point[2]), 5),
                    'raw_depth_3x3_m': np.round(patch, 4).tolist(),
                    'median_m': round(float(np.median(patch)), 5),
                })
            print(
                f'{robot_id}: candidate_{candidate_index} nine PnP depth samples='
                f'{measurements}',
                flush=True,
            )

        marker_rotation = _rotation_y(-math.pi / 2)
        camera_rotation = _rotation_z(-math.pi / 2) @ _rotation_x(-math.pi / 2)
        half = core.marker_size_m / 2
        marker_corners = np.array([
            (-half, half, 0.), (half, half, 0.),
            (half, -half, 0.), (-half, -half, 0.),
        ])
        world_corners = MARKER_CENTER + marker_corners @ marker_rotation.T
        camera_corners = (world_corners - CAMERA_ORIGIN[robot_id]) @ camera_rotation
        projected = (
            camera_corners[:, :2] / camera_corners[:, 2, None]
            * np.array([matrix[0, 0], matrix[1, 1]])
            + np.array([matrix[0, 2], matrix[1, 2]])
        )
        print(
            f'{robot_id}: nominal SDF 0.15 m corners projected through '
            f'ground-truth camera pose={np.round(projected, 2).tolist()} '
            f'expected_camera_z_m={np.round(camera_corners[:, 2], 5).tolist()}',
            flush=True,
        )

        center = np.mean(corners, axis=0).astype(int)
        for axis, fixed, start, end in (
                ('u', center[1], max(0, center[0] - 70), min(rgb.shape[1], center[0] + 71)),
                ('v', center[0], max(0, center[1] - 70), min(rgb.shape[0], center[1] + 71))):
            samples = []
            for value in range(start, end, 4):
                column, row = (value, fixed) if axis == 'u' else (fixed, value)
                samples.append(
                    f'{value}:gray={int(rgb[row, column].mean())},depth={depth[row, column]:.4f}')
            print(f'{robot_id}: {axis}-scan px: ' + ' | '.join(samples), flush=True)


def main():
    """Collect and report one ArUco RGB-D sample from each robot."""
    rclpy.init()
    node = GeometryProbe()
    deadline = time.monotonic() + 30
    try:
        while len(node.completed) < len(ROBOT_IDS) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.completed) != len(ROBOT_IDS):
            raise TimeoutError(
                f'no valid marker RGB-D sample from {set(ROBOT_IDS) - node.completed}')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
