#!/usr/bin/env python3
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
Check every robot's localization and navigation on the Task 1 map after the handover.

Used by tools/midterm_rehearsal.py --check-localization N. In TASK2A, the robots drive one
after another to N random goals each; the robot nearest the open arena (largest y) goes
first, so a robot in the start area never waits for a robot standing in front of it. A robot
that has no goal (boxed in by the others) tries again at the end of the round.
Goals lie on floor that is free both in the true geometry (a practice arena's truth_map,
or the midterm arena's pre-built map) and in the frozen Task 1 map Nav2 plans on, at least
goal_clearance_m from walls and from the other robots, and reachable on that map for the
robot past the robots standing elsewhere. A robot parked at
the goal must not cut the other robots off from each other or from most of the arena:
with the planner's clearance (0.265 m) and the padded peer outlines, a robot standing in a
doorway or between an obstacle and a wall closes it. The truth is only used to pick goals
and measure; the robots only receive the goal pose.

Measured against Gazebo's model poses (the base_footprint, ADR-0002):

  settled   the robot's own pose estimate (RobotState, AMCL on the Task 1 map) after it
            stopped at a goal: position and heading error
  moving    the same, sampled every 0.5 s while any robot drives; includes the age of
            the latest estimate (at 0.2 m/s, 0.1 s is 2 cm)
  arrival   distance of the true pose from the goal, and goals reached

A failed goal is retried once; when Nav2 found no path (error 208: the goal is only just
reachable past the parked robots, which the frozen map's cells and the peers' estimated
poses decide by a few centimetres), the retry gets a new goal and the report counts the
goal as replaced. Placement (place_waiting_robots) puts robots that waited
outside the arena at their handover poses, as the organizers would, after checking that
the footprints stay clear of the robots already there.
"""

import math
import random
import threading

import numpy
import yaml

# Robot footprint relative to base_footprint (edgeai_robot_navigation nav2 params).
FOOTPRINT = ((0.115, 0.175), (0.115, -0.175), (-0.315, -0.175), (-0.315, 0.175))
SWEEP_RADIUS_M = math.hypot(0.315, 0.175)
PEER_PADDING_M = 0.08          # peer_footprints padding_m
# Peers stand where their own AMCL estimates put them, up to about 0.1 m from the truth:
# goal reachability treats them as that much larger. Without it, a goal the truth just
# connected past a peer was unreachable for Nav2.
PEER_MARGIN_M = PEER_PADDING_M + 0.10
# NavFn's lethal distance (inscribed radius with footprint_padding, nav2_sim.yaml) plus
# one costmap cell for the rasterisation of walls and peers.
DRIVE_CLEARANCE_M = 0.265 + 0.05
GRID_M = 0.04                  # goal search grid; finer maps are coarsened to this


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def load_map(yaml_path):
    """Occupancy map as (blocked bool array [row 0 = lowest y], resolution, origin x, y)."""
    meta = yaml.safe_load(yaml_path.read_text())
    data = (yaml_path.parent / meta['image']).read_bytes()
    fields, offset = [], 0
    while len(fields) < 4:          # P5 header: magic, width, height, maxval
        while data[offset:offset + 1].isspace():
            offset += 1
        if data[offset:offset + 1] == b'#':
            offset = data.index(b'\n', offset)
            continue
        end = offset
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[offset:end])
        offset = end
    if fields[0] != b'P5':
        raise ValueError(f'{meta["image"]}: only binary PGM maps are supported')
    width, height, maxval = (int(f) for f in fields[1:])
    pixels = numpy.frombuffer(data, numpy.uint8, width * height, offset + 1)
    pixels = pixels.reshape(height, width)[::-1]
    occupancy = (maxval - pixels.astype(float)) / maxval
    if meta.get('negate', 0):
        occupancy = 1.0 - occupancy
    # Unknown cells count as blocked: a goal must be known free floor. A saved trinary map
    # writes unknown as 205, which free_thresh 0.25 would read as free.
    blocked = occupancy > meta['free_thresh']
    if meta.get('mode', 'trinary') == 'trinary':
        blocked |= pixels == 205
    return blocked, float(meta['resolution']), float(meta['origin'][0]), float(meta['origin'][1])


def merge(base, other):
    """The base grid with every cell blocked whose centre is blocked (or outside) in other."""
    blocked, resolution, ox, oy = base
    other_blocked, other_resolution, tx, ty = other
    rows, cols = numpy.indices(blocked.shape)
    i = numpy.floor((oy + (rows + 0.5) * resolution - ty) / other_resolution).astype(int)
    j = numpy.floor((ox + (cols + 0.5) * resolution - tx) / other_resolution).astype(int)
    inside = (i >= 0) & (i < other_blocked.shape[0]) & (j >= 0) & (j < other_blocked.shape[1])
    sampled = numpy.ones_like(blocked)
    sampled[inside] = other_blocked[i[inside], j[inside]]
    return blocked | sampled, resolution, ox, oy


def coarsen(grid, step=GRID_M):
    """Merge cells (a cell is blocked if any merged cell is) to about step metres."""
    blocked, resolution, ox, oy = grid
    factor = max(1, int(step / resolution + 1e-9))
    if factor == 1:
        return grid
    height, width = (s // factor * factor for s in blocked.shape)
    merged = blocked[:height, :width].reshape(height // factor, factor, width // factor, factor)
    return merged.any(axis=(1, 3)), resolution * factor, ox, oy


def with_robots(grid, poses, padding=PEER_MARGIN_M):
    """The grid with robot footprints (x, y, yaw) marked as blocked."""
    blocked, resolution, ox, oy = grid
    blocked = blocked.copy()
    for x, y, yaw in poses:
        outline = footprint(x, y, yaw, padding)
        xs, ys = [p[0] for p in outline], [p[1] for p in outline]
        for i in range(max(int((min(ys) - oy) / resolution), 0),
                       min(int((max(ys) - oy) / resolution) + 2, blocked.shape[0])):
            for j in range(max(int((min(xs) - ox) / resolution), 0),
                           min(int((max(xs) - ox) / resolution) + 2, blocked.shape[1])):
                if _inside((ox + (j + 0.5) * resolution, oy + (i + 0.5) * resolution),
                           outline):
                    blocked[i, j] = True
    return blocked, resolution, ox, oy


def _disc(grid, x, y, radius):
    blocked, resolution, ox, oy = grid
    rows, cols = numpy.indices(blocked.shape)
    inside = numpy.hypot(ox + (cols + 0.5) * resolution - x,
                         oy + (rows + 0.5) * resolution - y) <= radius
    return blocked | inside, resolution, ox, oy


def _grow(blocked, cells):
    grown = blocked.copy()
    height, width = blocked.shape
    for di in range(-cells, cells + 1):
        for dj in range(-cells, cells + 1):
            if di * di + dj * dj > cells * cells or (di == 0 and dj == 0):
                continue
            shifted = numpy.zeros_like(blocked)
            shifted[max(di, 0):height + min(di, 0), max(dj, 0):width + min(dj, 0)] = \
                blocked[max(-di, 0):height + min(-di, 0), max(-dj, 0):width + min(-dj, 0)]
            grown |= shifted
    return grown


def _reach(passable, grid, starts):
    """Passable cells 4-connected to the cells nearest to each start."""
    blocked, resolution, ox, oy = grid
    height, width = blocked.shape
    seen = numpy.zeros_like(blocked)
    queue = []
    for x, y in starts:
        i, j = int((y - oy) / resolution), int((x - ox) / resolution)
        # A start inside the inflation (next to a wall or a peer) begins at the nearest
        # passable cell.
        best = None
        for di in range(-10, 11):
            for dj in range(-10, 11):
                a, b = i + di, j + dj
                if 0 <= a < height and 0 <= b < width and passable[a, b]:
                    d = di * di + dj * dj
                    if best is None or d < best[0]:
                        best = (d, a, b)
        if best and not seen[best[1], best[2]]:
            seen[best[1], best[2]] = True
            queue.append((best[1], best[2]))
    while queue:
        i, j = queue.pop()
        for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= a < height and 0 <= b < width and passable[a, b] and not seen[a, b]:
                seen[a, b] = True
                queue.append((a, b))
    return seen


def _passable(grid, clearance_m):
    blocked, resolution = grid[0], grid[1]
    return ~_grow(blocked, int(math.ceil(clearance_m / resolution)))


def goal_cells(grid, starts, drive_clearance_m=DRIVE_CLEARANCE_M, goal_clearance_m=0.45):
    """Centres of cells reachable from starts that keep goal_clearance_m from obstacles."""
    blocked, resolution, ox, oy = grid
    seen = _reach(_passable(grid, drive_clearance_m), grid, starts)
    rows, cols = numpy.nonzero(seen & _passable(grid, goal_clearance_m))
    return [(ox + (j + 0.5) * resolution, oy + (i + 0.5) * resolution)
            for i, j in zip(rows.tolist(), cols.tolist())]


def choose_goal(grid, rng, robot, others, min_move_m=1.0, goal_clearance_m=0.45,
                keep_share=0.85, tries=40):
    """
    A random goal pose for robot (x, y, yaw) among the other robots' poses.

    The goal is reachable past the other robots, at least min_move_m away and
    goal_clearance_m from walls and robots; parked there (any heading, padded), the robot
    leaves every other robot at least keep_share of the floor it reaches now (a robot that
    closes a passage cuts the floor behind it off). None if no such goal is found.
    """
    occupied = with_robots(grid, others)
    cells = [c for c in goal_cells(occupied, [robot[:2]], goal_clearance_m=goal_clearance_m)
             if math.dist(c, robot[:2]) >= min_move_m]
    rng.shuffle(cells)
    # Per other robot: the grid with the remaining robots, and the floor it reaches while
    # this robot is away.
    views = []
    for index, other in enumerate(others):
        view = with_robots(grid, others[:index] + others[index + 1:])
        views.append((other[:2], view, int(_reach(_passable(view, DRIVE_CLEARANCE_M), view,
                                                  [other[:2]]).sum())))
    for x, y in cells[:tries]:
        if all(_reach(_passable(parked, DRIVE_CLEARANCE_M), parked, [start]).sum()
               >= keep_share * baseline
               for start, view, baseline in views
               for parked in [_disc(view, x, y, SWEEP_RADIUS_M + PEER_PADDING_M)]):
            return x, y, rng.uniform(-math.pi, math.pi)
    return None


def footprint(x, y, yaw, padding=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * (fx + math.copysign(padding, fx)) - s * (fy + math.copysign(padding, fy)),
             y + s * (fx + math.copysign(padding, fx)) + c * (fy + math.copysign(padding, fy)))
            for fx, fy in FOOTPRINT]


def _inside(point, polygon):
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _segment_distance(p, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(p[0] - ax - t * dx, p[1] - ay - t * dy)


def footprint_gap(first, second):
    """Smallest distance between two footprint polygons (0 when they overlap)."""
    if any(_inside(p, second) for p in first) or any(_inside(p, first) for p in second):
        return 0.0
    return min(_segment_distance(p, a, b)
               for polygon, other in ((first, second), (second, first))
               for p in polygon for a, b in zip(other, other[1:] + other[:1]))


def summarize(values):
    if not values:
        return None
    ordered = sorted(values)
    return {'mean': round(sum(ordered) / len(ordered), 3),
            'p95': round(ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))], 3),
            'max': round(ordered[-1], 3), 'samples': len(ordered)}


class GroundTruth:
    """Gazebo model poses and placement through gz-transport (GZ_PARTITION must be set)."""

    def __init__(self, world='empty'):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node as GzNode
        self._types = (Pose, Boolean)
        self.world = world
        self.node = GzNode()
        self.lock = threading.Lock()
        self.poses = {}
        if not self.node.subscribe(Pose_V, f'/world/{world}/dynamic_pose/info', self._on_poses):
            raise RuntimeError('cannot subscribe to the Gazebo pose topic')

    def _on_poses(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        with self.lock:
            for pose in msg.pose:
                if pose.name.startswith('robot_'):
                    self.poses[pose.name] = (stamp, pose.position.x, pose.position.y,
                                             pose.position.z, yaw_of(pose.orientation))

    def get(self, name):
        with self.lock:
            return self.poses.get(name)

    def place(self, name, x, y, z, yaw):
        Pose, Boolean = self._types
        request = Pose(name=name)
        request.position.x, request.position.y, request.position.z = x, y, z
        request.orientation.z, request.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        # With the robot models loaded, the reply often times out although Gazebo moves the
        # model; the caller confirms the move from the published poses instead.
        self.node.request(f'/world/{self.world}/set_pose', request, Pose, Boolean, 1000)


def place_waiting_robots(truth, placements, robots, wait, min_gap_m=0.05):
    """Move waiting robots to their handover poses; refuse if a footprint would touch."""
    placed = []
    for robot, pose in placements.items():
        target = footprint(pose['x'], pose['y'], pose['yaw'])
        for other in robots:
            if other == robot:
                continue
            current = truth.get(other)
            if current is None:
                raise RuntimeError(f'no Gazebo pose for {other}')
            gap = footprint_gap(target, footprint(current[1], current[2], current[4]))
            if gap < min_gap_m:
                raise RuntimeError(
                    f'cannot place {robot} at ({pose["x"]}, {pose["y"]}): {gap:.3f} m from '
                    f'{other} at ({current[1]:.2f}, {current[2]:.2f})')
        z = truth.get(robot)[3]
        truth.place(robot, pose['x'], pose['y'], z, pose['yaw'])
        placed.append(robot)
    # Wait until Gazebo reports the new poses.
    wait(lambda: all(truth.get(r) and math.hypot(truth.get(r)[1] - placements[r]['x'],
                                                 truth.get(r)[2] - placements[r]['y']) < 0.03
                     for r in placed), 10, 'placement not confirmed by Gazebo')


class LocalizationCheck:
    """Drive the robots to random goals one at a time and measure localization."""

    def __init__(self, node, wait, received, truth, robots, grid, goals_per_robot,
                 seed=None, settle_s=1.5, goal_timeout_s=120.0):
        from edgeai_robot_interfaces.action import ExecuteNavigationGoal
        from rclpy.action import ActionClient
        self.Goal = ExecuteNavigationGoal.Goal
        self.node, self.wait, self.received, self.truth = node, wait, received, truth
        self.robots, self.grid, self.count = robots, coarsen(grid), goals_per_robot
        self.rng = random.Random(seed)
        self.settle_s, self.goal_timeout_s = settle_s, goal_timeout_s
        self.clients = {r: ActionClient(node, ExecuteNavigationGoal,
                                        f'/{r}/execute_navigation_goal') for r in robots}
        self.goals = []
        self.moving = {r: {'xy': [], 'yaw': []} for r in robots}
        self.sequence = 0

    def _estimate(self, robot):
        state = self.received.get(robot)
        if state is None or not state.localized:
            return None
        p = state.pose.pose
        stamp = state.header.stamp.sec + state.header.stamp.nanosec * 1e-9
        return stamp, p.position.x, p.position.y, yaw_of(p.orientation)

    def _error(self, robot):
        estimate, true = self._estimate(robot), self.truth.get(robot)
        if estimate is None or true is None:
            return None
        return (math.hypot(estimate[1] - true[1], estimate[2] - true[2]),
                abs(angle_diff(estimate[3], true[4])), true[0] - estimate[0])

    def _sample_moving(self):
        for robot in self.robots:
            error = self._error(robot)
            if error is not None:
                self.moving[robot]['xy'].append(error[0])
                self.moving[robot]['yaw'].append(math.degrees(error[1]))

    def _drive(self, robot, goal):
        state = self.received['state']
        self.sequence += 1
        request = self.Goal()
        request.run_id, request.map_id = state.run_id, state.map_id
        request.map_revision = state.map_revision
        request.request_id = f'localization-check-{self.sequence}'
        request.purpose = self.Goal.PURPOSE_TRANSIT
        request.goal_pose.header.frame_id = 'map'
        request.goal_pose.header.stamp = self.node.get_clock().now().to_msg()
        request.goal_pose.pose.position.x, request.goal_pose.pose.position.y = goal[:2]
        request.goal_pose.pose.orientation.z = math.sin(goal[2] / 2)
        request.goal_pose.pose.orientation.w = math.cos(goal[2] / 2)
        request.xy_tolerance_m, request.yaw_tolerance_rad = 0.15, 0.35
        request.timeout.sec = int(self.goal_timeout_s)
        client = self.clients[robot]
        self.wait(lambda: client.server_is_ready(), 10, f'{robot} navigation action unavailable')
        sent = client.send_goal_async(request)
        self.wait(sent.done, 10, f'{robot} goal not answered')
        handle = sent.result()
        if not handle.accepted:
            return False, 'rejected (see the robot_agent log)'
        result = handle.get_result_async()
        clock = {'next': 0.0}

        def done():
            now = self.node.get_clock().now().nanoseconds * 1e-9
            if now >= clock['next']:
                clock['next'] = now + 0.5
                self._sample_moving()
            return result.done()
        self.wait(done, self.goal_timeout_s + 15, f'{robot} goal did not finish')
        outcome = result.result().result
        return outcome.success, outcome.reason.detail

    def _settle(self):
        end = self.node.get_clock().now().nanoseconds * 1e-9 + self.settle_s
        self.wait(lambda: self.node.get_clock().now().nanoseconds * 1e-9 >= end,
                  self.settle_s * 4 + 5, 'simulation clock stalled')

    def run(self, say):
        for round_index in range(self.count):
            order = sorted(self.robots, key=lambda r: -self.truth.get(r)[2])
            deferred = set()
            while order:
                robot = order.pop(0)
                here = self.truth.get(robot)
                others = [(p[1], p[2], p[4]) for p in
                          (self.truth.get(o) for o in self.robots if o != robot)]
                goal = choose_goal(self.grid, self.rng, (here[1], here[2], here[4]), others)
                if goal is None and robot not in deferred:
                    deferred.add(robot)
                    order.append(robot)
                    continue
                record = {'robot': robot, 'round': round_index + 1, 'attempts': 0,
                          'deferred': robot in deferred,
                          'goal': None if goal is None else [round(v, 3) for v in goal]}
                self.goals.append(record)
                if goal is None:
                    record['error'] = 'no free goal away from the other robots'
                    continue
                for _ in range(2):
                    record['attempts'] += 1
                    success, detail = self._drive(robot, goal)
                    if success:
                        break
                    record['failure'] = detail
                    self._settle()
                    if 'error 208' in detail:
                        here = self.truth.get(robot)
                        others = [(p[1], p[2], p[4]) for p in
                                  (self.truth.get(o) for o in self.robots if o != robot)]
                        replacement = choose_goal(self.grid, self.rng,
                                                  (here[1], here[2], here[4]), others)
                        if replacement is not None:
                            record['replaced_goal'] = record['goal']
                            goal = replacement
                            record['goal'] = [round(v, 3) for v in goal]
                record['reached'] = success
                self._settle()
                error = self._error(robot)
                true = self.truth.get(robot)
                record['arrival_m'] = round(math.hypot(true[1] - goal[0], true[2] - goal[1]), 3)
                if error is not None:
                    record['settled_xy_m'] = round(error[0], 3)
                    record['settled_yaw_deg'] = round(math.degrees(error[1]), 2)
                say('localization check goal', **record)

    def report(self, max_settled_xy_m=0.10, max_settled_yaw_deg=5.0, max_moving_xy_m=0.25):
        robots = {}
        for robot in self.robots:
            mine = [g for g in self.goals if g['robot'] == robot]
            robots[robot] = {
                'goals': len(mine),
                'reached': sum(1 for g in mine if g.get('reached')),
                'retries': sum(g['attempts'] - 1 for g in mine if g['attempts'] > 1),
                'replaced_goals': sum(1 for g in mine if 'replaced_goal' in g),
                'settled_xy_m': summarize([g['settled_xy_m'] for g in mine
                                           if 'settled_xy_m' in g]),
                'settled_yaw_deg': summarize([g['settled_yaw_deg'] for g in mine
                                              if 'settled_yaw_deg' in g]),
                'moving_xy_m': summarize(self.moving[robot]['xy']),
                'moving_yaw_deg': summarize(self.moving[robot]['yaw']),
            }
        problems = []
        for robot, values in robots.items():
            if values['reached'] < values['goals'] or values['goals'] < self.count:
                problems.append(f'{robot}: reached {values["reached"]} of {self.count} goals')
            for key, limit in (('settled_xy_m', max_settled_xy_m),
                               ('settled_yaw_deg', max_settled_yaw_deg),
                               ('moving_xy_m', max_moving_xy_m)):
                if values[key] is None:
                    problems.append(f'{robot}: no {key} samples')
                elif values[key]['max'] > limit:
                    problems.append(f'{robot}: {key} max {values[key]["max"]} > {limit}')
        return {'passed': not problems, 'problems': problems,
                'limits': {'settled_xy_m': max_settled_xy_m,
                           'settled_yaw_deg': max_settled_yaw_deg,
                           'moving_xy_m': max_moving_xy_m},
                'robots': robots, 'goals': self.goals}
