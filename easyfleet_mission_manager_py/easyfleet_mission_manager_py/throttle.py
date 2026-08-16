# Copyright 2026 Intelligent Robotics Lab
#
# This file is part of the project EasyFleet
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

"""Rate limiter, mirroring easyfleet_mission_manager's throttle.hpp."""

import threading
import time


class Throttle:
    """
    Minimum-interval rate limiter.

    Unlike the C++ original (which uses a shared_ptr<time_point> so
    copies of the same Throttle share one clock, since a feedback
    callback captures it by value into a std::function), a Python
    Throttle instance captured by a closure is already shared by
    reference -- no extra indirection needed for the same effect.
    """

    def __init__(self, interval_sec: float):
        self._interval_sec = interval_sec
        self._last = 0.0
        self._lock = threading.Lock()

    def ready(self) -> bool:
        """Return whether at least `interval_sec` has elapsed since the last True."""
        now = time.monotonic()
        with self._lock:
            if now - self._last >= self._interval_sec:
                self._last = now
                return True
            return False
