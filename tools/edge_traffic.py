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
Measure the ROS 2 traffic that would cross between the robots and the edge node.

Run it next to a running system (same ROS_DOMAIN_ID, e.g. during `pixi run practice`
with --domain 77). It reads the ROS graph, puts every node on the robot side
(/robot_N/..., and the simulation's /clock bridge) or the edge side (everything else),
and subscribes to each topic that has a publisher on one side and a subscriber on the
other. It reports messages and bytes per second for each such topic, per direction.

    pixi run bash -c 'source install/setup.bash && ROS_DOMAIN_ID=77 \\
        python tools/edge_traffic.py --seconds 60 --out traffic.json'

Subscriptions are best effort (compatible with every publisher) and take serialized
messages, so the probe itself costs little; under heavy load it may miss messages and
underestimate. Services and action goals are not counted (requests are small and rare).
"""

import argparse
import json
import os
import re
import time

ROBOT_NODE = re.compile(r'^/robot_\d+(/|_|$)')
IGNORED_NODE = re.compile(r'^/(launch_ros_|transform_listener_impl_|edge_traffic$)')
SIM_ROBOT_NODES = {'/clock_bridge'}     # Gazebo runs with the robots


def side(node):
    """Return 'robot', 'edge' or None (helper nodes that do not count) for a node name."""
    if IGNORED_NODE.match(node):
        return None
    if ROBOT_NODE.match(node) or node in SIM_ROBOT_NODES:
        return 'robot'
    return 'edge'


def crossing(graph):
    """Map each topic with publishers and subscribers on different sides to its details."""
    topics = {}
    for node, info in graph.items():
        where = side(node)
        if where is None:
            continue
        for kind in ('pub', 'sub'):
            for topic, kind_type in info[kind].items():
                entry = topics.setdefault(topic, {'type': kind_type, 'pub': {}, 'sub': {}})
                entry[kind].setdefault(where, []).append(node)
    result = {}
    for topic, entry in topics.items():
        directions = [f'{a}->{b}' for a, b in (('robot', 'edge'), ('edge', 'robot'))
                      if a in entry['pub'] and b in entry['sub']]
        if directions:
            result[topic] = {'type': entry['type'], 'directions': directions,
                             'publishers': entry['pub'], 'subscribers': entry['sub']}
    return result


def read_graph(node, settle_s):
    end = time.monotonic() + settle_s
    import rclpy
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    graph = {}
    for name, namespace in node.get_node_names_and_namespaces():
        full = namespace.rstrip('/') + '/' + name
        try:
            pubs = node.get_publisher_names_and_types_by_node(name, namespace)
            subs = node.get_subscriber_names_and_types_by_node(name, namespace)
        except Exception:       # the node left while we asked
            continue
        graph[full] = {'pub': {t: types[0] for t, types in pubs},
                       'sub': {t: types[0] for t, types in subs}}
    return graph


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=60.0, help='measuring time [s wall]')
    parser.add_argument('--settle', type=float, default=8.0,
                        help='time for graph discovery before measuring [s]')
    parser.add_argument('--out', default=None, help='write the full report as JSON')
    options = parser.parse_args()
    os.environ.setdefault('CYCLONEDDS_URI', '<CycloneDDS><Domain><Discovery>'
                          '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
                          '</Discovery></Domain></CycloneDDS>')

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = rclpy.create_node('edge_traffic')
    graph = read_graph(node, options.settle)
    topics = crossing(graph)
    counts = {topic: [0, 0] for topic in topics}
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    for topic, entry in topics.items():
        def count(data, topic=topic):
            counts[topic][0] += 1
            counts[topic][1] += len(data)
        node.create_subscription(get_message(entry['type']), topic, count, qos, raw=True)
    started = time.monotonic()
    while time.monotonic() - started < options.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.monotonic() - started
    rows = []
    for topic, entry in topics.items():
        messages, size = counts[topic]
        entry.update(hz=round(messages / elapsed, 2), kib_per_s=round(size / elapsed / 1024, 2),
                     bytes_per_msg=round(size / messages) if messages else 0)
        rows.append((entry['kib_per_s'], topic, entry))
    rows.sort(reverse=True)
    total = {d: round(sum(e['kib_per_s'] for _, _, e in rows if d in e['directions']), 1)
             for d in ('robot->edge', 'edge->robot')}
    print(f'{"KiB/s":>9} {"Hz":>8} {"B/msg":>8}  direction        topic')
    for kib, topic, entry in rows:
        print(f'{kib:9.2f} {entry["hz"]:8.2f} {entry["bytes_per_msg"]:8d}  '
              f'{"+".join(entry["directions"]):16} {topic}')
    print('total KiB/s:', total)
    if options.out:
        with open(options.out, 'w', encoding='utf-8') as stream:
            json.dump({'seconds': round(elapsed, 1), 'total_kib_per_s': total,
                       'topics': topics, 'graph': graph}, stream, indent=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
