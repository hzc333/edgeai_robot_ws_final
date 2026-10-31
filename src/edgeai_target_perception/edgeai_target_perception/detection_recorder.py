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

"""
Detection evidence for the midterm check (Part 1).

The midterm asks for a record of every successful detection: marker ID, target
type, position relative to the camera or robot, and detection time, plus visual
evidence. This node subscribes to each robot's raw TargetObservation and camera
image and writes, under output_dir/<run_id>/:

* detections.jsonl  one line per (robot, marker) at most every record_period_s;
* <target>_<robot>_<sim time>.png  the camera image with the projected marker
  outline and its identity, on the first detection of each marker and then at
  most every image_period_s;
* summary.json  first and latest detection per target, also published latched
  on /edge/detection_summary (std_msgs/String, JSON).

Positions are the marker centre in the observation's camera optical frame
(x right, y down, z forward), in metres; times are the image (simulation) time.
"""

import json
import math
import os
from pathlib import Path

from cv_bridge import CvBridge
from edgeai_robot_interfaces.msg import TargetObservation
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

TYPE_NAMES = {TargetObservation.TYPE_NORMAL: 'normal',
              TargetObservation.TYPE_COOPERATIVE: 'cooperative'}
IMAGE_MATCH_S = 0.1


def rotation_from_quaternion(x, y, z, w):
    """Return the 3x3 rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def observation_record(message):
    """Midterm detection record of one TargetObservation (JSON-serialisable)."""
    position = message.marker_pose.pose.position
    stamp = message.header.stamp
    return {
        'sim_time': stamp.sec + stamp.nanosec * 1e-9,
        'run_id': message.run_id,
        'robot_id': message.robot_id,
        'marker_id': int(message.marker_id),
        'target_id': message.target_id,
        'target_type': TYPE_NAMES.get(message.target_type, 'unknown'),
        'frame_id': message.header.frame_id,
        'position_m': [round(position.x, 4), round(position.y, 4), round(position.z, 4)],
        'range_m': round(math.sqrt(position.x ** 2 + position.y ** 2 + position.z ** 2), 4),
        'confidence': round(float(message.confidence), 3),
    }


def project_marker_outline(pose, k, marker_size_m):
    """Pixel corners (TL, TR, BR, BL) of a marker whose pose is in the optical frame."""
    o = pose.orientation
    rotation = rotation_from_quaternion(o.x, o.y, o.z, o.w)
    centre = np.array([pose.position.x, pose.position.y, pose.position.z])
    half = marker_size_m / 2.0
    corners = []
    for mx, my in ((-half, half), (half, half), (half, -half), (-half, -half)):
        x, y, z = rotation @ np.array([mx, my, 0.0]) + centre
        if z <= 0.0:
            return None
        corners.append((k[0] * x / z + k[2], k[4] * y / z + k[5]))
    return corners


class DetectionRecorder(Node):

    def __init__(self, **kwargs):
        super().__init__('detection_recorder', **kwargs)
        p = self.declare_parameter
        robots = [r.strip() for r in p('robot_ids', 'robot_1').value.split(',') if r.strip()]
        self.output_dir = Path(p('output_dir', 'runs/detections').value)
        self.record_period_s = float(p('record_period_s', 1.0).value)
        self.image_period_s = float(p('image_period_s', 5.0).value)
        self.marker_size_m = float(p('marker_size_m', 0.16).value)
        self.bridge = CvBridge()
        self.images = {}            # robot -> latest Image
        self.camera_info = {}       # robot -> latest CameraInfo
        self.last_record = {}       # (robot, marker) -> sim time
        self.last_image = {}        # (robot, marker) -> sim time
        self.summary = {}           # run -> target -> summary
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.summary_pub = self.create_publisher(String, '/edge/detection_summary', latched)
        for robot in robots:
            self.create_subscription(
                TargetObservation, f'/{robot}/target_observations', self._on_observation,
                qos_profile_sensor_data)
            self.create_subscription(
                Image, f'/{robot}/camera/image',
                lambda m, r=robot: self.images.__setitem__(r, m), qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, f'/{robot}/camera/camera_info',
                lambda m, r=robot: self.camera_info.__setitem__(r, m), qos_profile_sensor_data)
        self.get_logger().info(f'recording detections of {robots} under {self.output_dir}')

    def _run_dir(self, run_id):
        path = self.output_dir / (run_id or 'no-run')
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _on_observation(self, message):
        record = observation_record(message)
        key = (record['robot_id'], record['marker_id'])
        now = record['sim_time']
        first = key not in self.last_record
        if not first and now - self.last_record[key] < self.record_period_s:
            return
        self.last_record[key] = now
        run_dir = self._run_dir(record['run_id'])
        with open(run_dir / 'detections.jsonl', 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(record) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        if first:
            self.get_logger().info(
                f"detected {record['target_id']} (marker {record['marker_id']}, "
                f"{record['target_type']}) by {record['robot_id']} at sim time "
                f"{now:.2f} s: camera-frame position {record['position_m']} m, "
                f"range {record['range_m']:.2f} m")
        if first or now - self.last_image.get(key, -math.inf) >= self.image_period_s:
            if self._save_image(message, record, run_dir):
                self.last_image[key] = now
        self._update_summary(record, run_dir)

    def _save_image(self, message, record, run_dir):
        image = self.images.get(record['robot_id'])
        info = self.camera_info.get(record['robot_id'])
        if image is None or info is None:
            return False
        if abs((Time.from_msg(image.header.stamp) - Time.from_msg(message.header.stamp))
               .nanoseconds) * 1e-9 > IMAGE_MATCH_S:
            return False
        import cv2
        frame = self.bridge.imgmsg_to_cv2(image, 'bgr8').copy()
        corners = project_marker_outline(message.marker_pose.pose, info.k, self.marker_size_m)
        if corners is not None:
            points = np.array(corners, dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(frame, [points], True, (0, 220, 0), 2)
        label = (f"{record['target_id']} id={record['marker_id']} {record['target_type']} "
                 f"r={record['range_m']:.2f}m t={record['sim_time']:.1f}s")
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(frame, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 1)
        name = f"{record['target_id']}_{record['robot_id']}_{record['sim_time']:09.2f}.png"
        cv2.imwrite(str(run_dir / name), frame)
        return True

    def _update_summary(self, record, run_dir):
        targets = self.summary.setdefault(record['run_id'], {})
        entry = targets.setdefault(record['target_id'], {
            'marker_id': record['marker_id'], 'target_type': record['target_type'],
            'first': record, 'records': 0})
        entry['latest'] = record
        entry['records'] += 1
        document = {'run_id': record['run_id'], 'targets': targets}
        text = json.dumps(document, indent=2)
        (run_dir / 'summary.json').write_text(text + '\n', encoding='utf-8')
        self.summary_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = DetectionRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
