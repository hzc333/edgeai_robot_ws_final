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

"""Admission geometry; actual path following and braking remain with the robots."""

import math


def footprint(x, y, yaw, settings):
    """Map-frame rectangular footprint, matching the configured robot envelope."""
    c, s = math.cos(yaw), math.sin(yaw)
    f, r, w = (settings.footprint_front_m, settings.footprint_rear_m,
               settings.footprint_half_width_m)
    return [(x + c * px - s * py, y + s * px + c * py)
            for px, py in ((f, w), (-r, w), (-r, -w), (f, -w))]


def hull(points):
    """Convex hull, including a translated straight-entry footprint envelope."""
    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    points = sorted(set(points))
    halves = []
    for ordered in (points, list(reversed(points))):
        half = []
        for p in ordered:
            while len(half) >= 2 and cross(half[-2], half[-1], p) <= 0:
                half.pop()
            half.append(p)
        halves.extend(half[:-1])
    return halves


def polygon_gap(first, second):
    """Euclidean edge distance; zero for touching or overlapping convex polygons."""
    separated = False
    for poly in (first, second):
        for a, b in zip(poly, poly[1:] + poly[:1]):
            axis = (a[1] - b[1], b[0] - a[0])
            one = [p[0] * axis[0] + p[1] * axis[1] for p in first]
            two = [p[0] * axis[0] + p[1] * axis[1] for p in second]
            if max(one) < min(two) or max(two) < min(one):
                separated = True
    if not separated:
        return 0.0

    def point_edge(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        projection = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, projection))
        return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)

    return min(point_edge(p, a, b)
               for points, edges in ((first, second), (second, first))
               for p in points for a, b in zip(edges, edges[1:] + edges[:1]))


def entry_heading(target, index):
    """Return the entry heading; the robot faces its destination, final yaw is free."""
    slot = target.slots[index]
    if target.approaches:
        approach = target.approaches[index]
        if math.dist(approach[:2], slot[:2]) > 1e-6:
            return math.atan2(slot[1] - approach[1], slot[0] - approach[0])
    return slot[2]


def entry_envelope(target, index, settings, robot=None):
    """Nominal straight final entry, or actual footprint if already parked.

    This is an admission check, not a prediction of Nav2's global path, turns
    or tracking error. The robots must still avoid obstacles along their actual motion.
    """
    slot = target.slots[index]
    if robot and math.dist((robot.x, robot.y), slot[:2]) <= settings.arrival_tolerance_m:
        return footprint(robot.x, robot.y, robot.yaw, settings)
    heading = entry_heading(target, index)
    points = footprint(*slot[:2], heading, settings)
    if target.approaches:
        points += footprint(*target.approaches[index][:2], heading, settings)
    return hull(points)


def entry_conflict(target, index, robot, peers, settings):
    """Return the peer obstructing the nominal entry, or an empty string."""
    envelope = entry_envelope(target, index, settings, robot)
    for peer in peers:
        occupied = footprint(peer.x, peer.y, peer.yaw, settings)
        if polygon_gap(envelope, occupied) <= settings.entry_clearance_m:
            return peer.robot_id
    return ''


def validate_layout(target, settings):
    """Reject overlapping slots and an entry cutting across an earlier slot."""
    parked = [footprint(*slot[:2], entry_heading(target, i), settings)
              for i, slot in enumerate(target.slots)]
    margin = settings.entry_clearance_m + 2 * settings.arrival_tolerance_m
    for i, occupied in enumerate(parked):
        for j in range(i + 1, len(parked)):
            if polygon_gap(occupied, parked[j]) <= margin:
                raise ValueError(f'{target.target_id}: parking slots have insufficient clearance')
            if polygon_gap(occupied, entry_envelope(target, j, settings)) <= margin:
                raise ValueError(f'{target.target_id}: later entry crosses an earlier parking slot')
