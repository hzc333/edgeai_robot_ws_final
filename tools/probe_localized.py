"""Debug helper: why does robot_agent report a robot as not localized?

Prints, per robot, how many amcl_pose messages arrive in 5 s (with the last
covariance) and the age of the latest map -> base_footprint transform.
"""
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


def main():
    robots = sys.argv[1:] or ['robot_1', 'robot_2']
    rclpy.init()
    node = rclpy.create_node('probe_localized',
                             parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    TransformListener(buffer, node)
    poses = {r: [] for r in robots}
    for r in robots:
        node.create_subscription(PoseWithCovarianceStamped, f'/{r}/amcl_pose',
                                 lambda m, r=r: poses[r].append(m), 10)
    end = time.monotonic() + 5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    now = node.get_clock().now().nanoseconds * 1e-9
    for r in robots:
        try:
            t = buffer.lookup_transform('map', f'{r}/base_footprint', Time(),
                                        timeout=Duration(seconds=0.5))
            age = now - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
            tf = f'age {age:.2f} s'
        except Exception as error:  # noqa: BLE001
            tf = f'lookup failed: {error}'
        last = poses[r][-1].pose.covariance if poses[r] else None
        cov = (f'xy std {last[0] ** 0.5:.3f} yaw std {last[35] ** 0.5:.3f}'
               if last else 'none')
        print(f'{r}: amcl_pose in 5 s: {len(poses[r])} ({cov}); map->base {tf}')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
