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

"""LinkWatchdog, mirroring easyfleet_core's link_watchdog.hpp/.cpp."""

import threading

from easyfleet_interfaces.msg import CapabilityStatus


class LinkWatchdog:
    """
    Whether the fleet's heartbeat is still heard, with a timeout.

    States are CapabilityStatus.LINK_*: NOT_MONITORED with a timeout of 0
    (or less), WAITING until the first heartbeat, then OK or LOST depending
    on the age of the last one. Times are monotonic seconds. Thread-safe.
    """

    def __init__(self, timeout_sec: float = 0.0):
        self._lock = threading.Lock()
        self.reset(timeout_sec)

    def reset(self, timeout_sec: float) -> None:
        """Change the timeout; the link is WAITING again."""
        with self._lock:
            self._last_heard = None
            if timeout_sec > 0.0:
                self._timeout = float(timeout_sec)
                self._state = CapabilityStatus.LINK_WAITING
            else:
                self._timeout = None
                self._state = CapabilityStatus.LINK_NOT_MONITORED

    def heard(self, now: float) -> None:
        """Record a heartbeat heard at `now`."""
        with self._lock:
            if self._timeout is None:
                return
            if self._last_heard is None or now > self._last_heard:
                self._last_heard = now
            self._state = CapabilityStatus.LINK_OK

    def update(self, now: float) -> int:
        """Recompute and return the state."""
        with self._lock:
            if self._timeout is not None and self._last_heard is not None:
                self._state = (
                    CapabilityStatus.LINK_LOST if now - self._last_heard > self._timeout
                    else CapabilityStatus.LINK_OK)
            return self._state

    @property
    def state(self) -> int:
        """Return the state as of the last heard() or update()."""
        with self._lock:
            return self._state

    @property
    def monitored(self) -> bool:
        """Return whether a timeout is set."""
        with self._lock:
            return self._timeout is not None
