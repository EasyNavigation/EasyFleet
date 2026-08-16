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

"""RobotHandle, mirroring easyfleet_mission_manager's robot_handle.hpp/.cpp."""

import threading
import time

from . import ansi
from .capability_info import print_capability_info, print_capability_summary_line
from .capability_state import CapabilityState
from .detail.running_capability import RunningCapability
from .output import safe_print

# How long without a heartbeat before is_alive() gives up on a capability
# -- a few times the 1 Hz heartbeat period a capability itself publishes
# at, so one or two dropped messages don't flip it.
_ALIVE_TIMEOUT_SEC = 3.0

# Default: how long a mission script lets a capability run before
# stopping it, if it hasn't finished on its own by then.
RUN_TIMEOUT_SEC = 10.0


class MissionManagerError(RuntimeError):
    """Caller-usage error on a RobotHandle, mirroring easyfleet::RobotHandle's std::logic_error."""


class _RunningEntry:
    __slots__ = ('action_type', 'running')

    def __init__(self, action_type, running: RunningCapability):
        self.action_type = action_type
        self.running = running


class RobotHandle:
    """
    A remote robot's capabilities, as seen and commanded from a mission script.

    A thin proxy over whatever this robot announced on /capabilities,
    discovered by the FleetSession it's added to. Owns no ROS nodes of
    its own.

    Usage:
        controller = SimpleController()
        robot_1 = RobotHandle('robot_1')
        controller.add_robot(robot_1)
        controller.discover_capabilities()

        if not robot_1.has_capability('navigation'):
            ...

        robot_1.run_capability('navigation', Navigation, make_navigation_goal('kitchen'))
        while robot_1.is_capability_running('navigation'):
            controller.spin_some()
        if robot_1.capability_state('navigation') == CapabilityState.FAILED:
            ...
    """

    def __init__(self, name: str):
        self._name = name
        self._capabilities = []
        self._running: dict[str, _RunningEntry] = {}
        self._last_heartbeat: dict[str, float] = {}
        self._session = None
        self._lock = threading.Lock()

    @property
    def name(self) -> str:
        return self._name

    def has_capability(self, capability_type: str) -> bool:
        """Return whether this robot announced an *active* capability of this type."""
        return any(
            info.capability == capability_type and info.active
            for info in self._capabilities)

    def run_capability(
        self, capability_type: str, action_type, goal=None, timeout_sec=RUN_TIMEOUT_SEC,
    ):
        """
        Send `goal` to `capability_type` and return immediately.

        `action_type` is the easyfleet_interfaces.action module (e.g.
        Navigation) this capability_type talks -- fixed by the *first*
        run_capability() call for a given capability_type on this handle;
        a later call with a *different* action_type for the same
        capability_type is a caller bug and raises MissionManagerError.

        `goal` defaults to a default-constructed `action_type.Goal()`.
        """
        if self._session is None:
            raise MissionManagerError(
                f'RobotHandle.run_capability("{capability_type}", ...): this handle was '
                'never added to a FleetSession (call FleetSession.add_robot()/'
                'SimpleController.add_robot() first).')

        info = next(
            (i for i in self._capabilities if i.capability == capability_type and i.active),
            None)
        if info is None:
            safe_print(
                f'{ansi.YELLOW}[{self._name}/{capability_type}] not active, skipping.'
                f'{ansi.RESET}')
            return

        if goal is None:
            goal = action_type.Goal()

        with self._lock:
            entry = self._running.get(capability_type)
            if entry is None:
                running = RunningCapability(
                    self._session.node, action_type, info.action_name, self._name,
                    capability_type, self._session.status_marker_publisher)
                entry = _RunningEntry(action_type, running)
                self._running[capability_type] = entry
            elif entry.action_type is not action_type:
                raise MissionManagerError(
                    f'RobotHandle.run_capability("{capability_type}", ...): this capability '
                    'type was already run with a different action type on this handle -- a '
                    'capability_type must mean the same action type for the life of a '
                    'RobotHandle.')

        entry.running.run(goal, timeout_sec)

    def stop_capability(self, capability_type: str) -> None:
        """Ask `capability_type` to stop whatever it's doing, if anything."""
        with self._lock:
            entry = self._running.get(capability_type)
        if entry is not None:
            entry.running.stop()

    def is_capability_running(self, capability_type: str) -> bool:
        return self.capability_state(capability_type) == CapabilityState.RUNNING

    def capability_state(self, capability_type: str) -> CapabilityState:
        with self._lock:
            entry = self._running.get(capability_type)
        if entry is None:
            return CapabilityState.IDLE
        return entry.running.state

    def is_alive(self, capability_type: str) -> bool:
        """Return whether a heartbeat has been seen recently on /capabilities_status."""
        with self._lock:
            last = self._last_heartbeat.get(capability_type)
        if last is None:
            return False
        return (time.monotonic() - last) < _ALIVE_TIMEOUT_SEC

    def print_capabilities(self) -> None:
        for info in self._capabilities:
            print_capability_summary_line(info)
        for info in self._capabilities:
            safe_print('')
            print_capability_info(info)

    @property
    def capabilities(self) -> list:
        return list(self._capabilities)

    # --- Internal: called by FleetSession only, not part of the public API ---

    def _attach(self, session) -> None:
        self._session = session

    def _set_capabilities(self, capabilities: list) -> None:
        self._capabilities = capabilities

    def _note_heartbeat(self, capability_type: str, when: float) -> None:
        with self._lock:
            self._last_heartbeat[capability_type] = when

    def _check_timeouts(self) -> None:
        with self._lock:
            entries = list(self._running.values())
        for entry in entries:
            entry.running.check_timeout()
