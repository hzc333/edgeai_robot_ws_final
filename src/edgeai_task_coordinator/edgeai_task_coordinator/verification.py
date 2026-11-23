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

"""Official target-centred, simultaneous continuous-dwell verification."""

import math


class DwellVerifier:
    """Credit only fresh paired samples; never integrate a cached pose forward."""

    def __init__(self):
        self.start = None
        self.last_time = None
        self.last_sample = None
        self.completed = False
        self.elapsed = 0.0

    def reset(self):
        self.start = None
        self.last_sample = None
        self.elapsed = 0.0

    def update(self, now, robots, target, settings, valid=True, *, zones=None):
        """Return (completed, reset, distances) at the latest common sample time."""
        previous_start = self.start
        centers = zones if zones is not None else [target.center] * len(robots)
        distances = {r.robot_id: math.dist((r.x, r.y), center)
                     for r, center in zip(robots, centers)}
        rewind = self.last_time is not None and now < self.last_time
        self.last_time = now
        stamps = [r.stamp for r in robots]
        inside = (valid and not rewind and len(robots) == target.required
                  and len(distances) == target.required
                  and all(r.healthy(now, settings.state_stale_s) for r in robots)
                  and max(stamps) - min(stamps) <= settings.maximum_state_skew_s
                  and all(d <= target.radius for d in distances.values()))
        if self.completed:
            return True, False, distances
        if not inside:
            self.reset()
            return False, previous_start is not None, distances
        sample = min(stamps)
        if (self.last_sample is not None
                and (sample < self.last_sample
                     or sample - self.last_sample > settings.state_stale_s)):
            self.reset()
        if self.start is None:
            self.start = sample
        self.last_sample = sample
        self.elapsed = max(0.0, sample - self.start)
        self.completed = self.elapsed >= target.dwell
        reset = previous_start is not None and self.start != previous_start
        return self.completed, reset, distances
