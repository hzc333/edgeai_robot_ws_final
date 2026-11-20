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
Check that one robot provides what the software stack expects from its drivers.

Run it on the robot, or on the edge node to also check the clocks, while the drivers
run (the stack may run too; the robot must stand still):

    pixi run bash -c 'source install/setup.bash && python tools/robot_preflight.py --robot robot_1'

Checks (docs/real_robot_checklist.md lists where each expectation comes from):

  topics   /<robot>/scan, odom, camera/image, camera/camera_info with their types
  rates    scan, odom and image at least the minimum rate
  frames   scan in <robot>/lidar_link, image in <robot>/camera_optical_frame, odom from
           <robot>/odom to <robot>/base_footprint
  tf       base_footprint -> lidar_link and -> camera_optical_frame exist; odom ->
           base_footprint is current
  lidar    the scan covers the full circle and most beams return a range
  camera   camera_info has intrinsics and matches the image size
  cmd_vel  a driver subscribes; at most one publisher, the safety_watchdog (ADR-0003)
  clock    message stamps are within --max-clock-offset of this machine's clock
           (skipped with --use-sim-time)

Exit status 1 when any check fails; warnings do not fail.
"""

import argparse
import json
import math
import os
import statistics
import time

MIN_RATE_HZ = {'scan': 5.0, 'odom': 20.0, 'image': 10.0}


class Report:
    def __init__(self):
        self.rows = []

    def add(self, level, check, detail):
        self.rows.append({'level': level, 'check': check, 'detail': detail})

    def failed(self):
        return any(r['level'] == 'FAIL' for r in self.rows)

    def print(self):
        for row in self.rows:
            print(f'{row["level"]:5} {row["check"]:8} {row["detail"]}')


def stamp_s(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def check_scan(report, robot, scans):
    if not scans:
        return
    scan = scans[-1]
    span = scan.angle_increment * (len(scan.ranges) - 1)
    if span < math.radians(350):
        report.add('FAIL', 'lidar', f'scan covers {math.degrees(span):.0f} deg, expected 360')
    valid = [r for r in scan.ranges if scan.range_min < r < scan.range_max]
    share = len(valid) / max(len(scan.ranges), 1)
    level = 'PASS' if share >= 0.5 else 'WARN'
    report.add(level, 'lidar', f'{len(scan.ranges)} beams, {share:.0%} return a range '
               f'({scan.range_min:.2f}-{scan.range_max:.1f} m)')
    expected = f'{robot}/lidar_link'
    report.add('PASS' if scan.header.frame_id == expected else 'FAIL', 'frames',
               f'scan frame {scan.header.frame_id!r} (expected {expected!r})')


def check_camera(report, robot, images, infos):
    if images:
        expected = f'{robot}/camera_optical_frame'
        frame = images[-1].header.frame_id
        report.add('PASS' if frame == expected else 'FAIL', 'frames',
                   f'image frame {frame!r} (expected {expected!r})')
    if not infos:
        report.add('FAIL', 'camera', 'no camera_info')
        return
    info = infos[-1]
    if not any(info.k) or info.k[0] <= 0 or info.k[4] <= 0:
        report.add('FAIL', 'camera', 'camera_info has no intrinsics (K is zero)')
    elif images and (info.width, info.height) != (images[-1].width, images[-1].height):
        report.add('FAIL', 'camera', f'camera_info {info.width}x{info.height} but image '
                   f'{images[-1].width}x{images[-1].height}')
    else:
        report.add('PASS', 'camera', f'{info.width}x{info.height}, fx {info.k[0]:.1f}, '
                   f'fy {info.k[4]:.1f}')


def check_odom(report, robot, odoms):
    if not odoms:
        return
    odom = odoms[-1]
    expected = (f'{robot}/odom', f'{robot}/base_footprint')
    got = (odom.header.frame_id, odom.child_frame_id)
    report.add('PASS' if got == expected else 'FAIL', 'frames',
               f'odom {got[0]!r} -> {got[1]!r} (expected {expected[0]!r} -> {expected[1]!r})')
    speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
    if speed > 0.02 or abs(odom.twist.twist.angular.z) > 0.05:
        report.add('WARN', 'odom', f'robot reports motion ({speed:.2f} m/s, '
                   f'{odom.twist.twist.angular.z:.2f} rad/s); checks assume it stands still')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--robot', default='robot_1')
    parser.add_argument('--seconds', type=float, default=5.0, help='listening time [s]')
    parser.add_argument('--use-sim-time', action='store_true',
                        help='simulation: skip the clock check')
    parser.add_argument('--max-clock-offset', type=float, default=0.1,
                        help='largest accepted |local clock - message stamp| [s]')
    parser.add_argument('--json', default=None, help='also write the report as JSON')
    options = parser.parse_args()
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node('robot_preflight')
    robot = options.robot
    topics = {'scan': (f'/{robot}/scan', LaserScan), 'odom': (f'/{robot}/odom', Odometry),
              'image': (f'/{robot}/camera/image', Image),
              'camera_info': (f'/{robot}/camera/camera_info', CameraInfo)}
    received = {key: [] for key in topics}
    arrivals = {key: [] for key in topics}
    offsets = []

    def keep(key):
        def callback(message):
            received[key] = received[key][-1:] + [message]   # newest two are enough
            now = time.time()
            arrivals[key].append(now)
            if hasattr(message, 'header') and not options.use_sim_time:
                offsets.append(now - stamp_s(message.header))
        return callback

    for key, (topic, kind) in topics.items():
        node.create_subscription(kind, topic, keep(key), qos_profile_sensor_data)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    end = time.monotonic() + options.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    report = Report()
    published = dict(node.get_topic_names_and_types())
    for key, (topic, kind) in topics.items():
        expected = f'{kind.__module__.split(".")[0]}/msg/{kind.__name__}'
        types = published.get(topic)
        if not node.count_publishers(topic):     # our own subscription lists the topic too
            report.add('FAIL', 'topics', f'{topic} not published')
        elif expected not in types:
            report.add('FAIL', 'topics', f'{topic} has type {types}, expected {expected}')
        elif not received[key]:
            report.add('FAIL', 'topics', f'{topic} published but nothing received')
        if key in MIN_RATE_HZ and len(arrivals[key]) >= 2:
            rate = (len(arrivals[key]) - 1) / (arrivals[key][-1] - arrivals[key][0])
            report.add('PASS' if rate >= MIN_RATE_HZ[key] else 'FAIL', 'rates',
                       f'{topic} {rate:.1f} Hz (at least {MIN_RATE_HZ[key]:.0f})')
    check_scan(report, robot, received['scan'])
    check_camera(report, robot, received['image'], received['camera_info'])
    check_odom(report, robot, received['odom'])

    base = f'{robot}/base_footprint'
    for child in (f'{robot}/lidar_link', f'{robot}/camera_optical_frame'):
        try:
            t = tf_buffer.lookup_transform(base, child, Time())
            p = t.transform.translation
            report.add('PASS', 'tf', f'{base} -> {child} at ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        except TransformException as error:
            report.add('FAIL', 'tf', f'{base} -> {child}: {error}')
    try:
        t = tf_buffer.lookup_transform(f'{robot}/odom', base, Time())
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp_s(t.header)
        level = 'PASS' if options.use_sim_time or abs(age) < 0.5 else 'FAIL'
        report.add(level, 'tf', f'{robot}/odom -> {base} present'
                   + ('' if options.use_sim_time else f', {age:.2f} s old'))
    except TransformException as error:
        report.add('FAIL', 'tf', f'{robot}/odom -> {base}: {error}')

    cmd_vel = f'/{robot}/cmd_vel'
    publishers = [i.node_name for i in node.get_publishers_info_by_topic(cmd_vel)]
    subscribers = [i.node_name for i in node.get_subscriptions_info_by_topic(cmd_vel)]
    report.add('PASS' if subscribers else 'FAIL', 'cmd_vel',
               f'subscribers {subscribers or "none (no driver listens)"}')
    if len(publishers) > 1 or any(n != 'safety_watchdog' for n in publishers):
        report.add('FAIL', 'cmd_vel', f'publishers {publishers}: only the safety_watchdog '
                   'may publish (ADR-0003)')
    else:
        report.add('PASS', 'cmd_vel', f'publishers {publishers or "none (stack not running)"}')

    if not options.use_sim_time:
        if offsets:
            median = statistics.median(offsets)
            level = 'PASS' if abs(median) <= options.max_clock_offset else 'FAIL'
            limit_ms = options.max_clock_offset * 1000
            report.add(level, 'clock', f'local clock - message stamp: median {median * 1000:.0f} '
                       f'ms over {len(offsets)} messages (limit {limit_ms:.0f} ms; includes '
                       'transport delay)')
        else:
            report.add('WARN', 'clock', 'no stamped messages to compare')

    report.print()
    if options.json:
        with open(options.json, 'w', encoding='utf-8') as stream:
            json.dump({'robot': robot, 'failed': report.failed(), 'checks': report.rows},
                      stream, indent=1)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if report.failed() else 0


if __name__ == '__main__':
    raise SystemExit(main())
