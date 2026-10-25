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

"""FleetSession, mirroring easyfleet_mission_manager's fleet_session.hpp/.cpp."""

import copy
import threading
import time

from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .capability_discovery import apply_description
from .capability_info import CapabilityInfo
from .mission_helpers import spin_in_background
from .status_markers import StatusMarkerPublisher


# How long without a heartbeat before a capability counts as dead -- a few
# times the 1 Hz heartbeat period, so one or two dropped messages don't flip it.
ALIVE_TIMEOUT_SEC = 3.0


class FleetSession:
    """
    Everything talking to a fleet from a mission-control process actually needs.

    Regardless of what decides *when* to command which robot: the ROS
    node and background-spinning executor every RobotHandle rides on,
    a live registry of every capability announced on /capabilities and
    /capabilities_status, the registry of added robots, and the
    automatic status markers RobotHandle.run_capability() publishes.

    `SimpleController` -- a plain mission script driving robots by hand
    -- is the simplest possible thing built on top of a FleetSession,
    but it's deliberately not the *only* thing that can be: a
    FleetSession has no notion of "how a mission decides what to do
    next", only "how to talk to the robots once something has decided".

    Usage:
        session = FleetSession()
        robot_1 = RobotHandle('robot_1')
        session.add_robot(robot_1)
        session.discover_capabilities()

        robot_1.run_capability('navigation', Navigation, make_navigation_goal('kitchen'))
        while robot_1.is_capability_running('navigation'):
            session.spin_some()

        session.shutdown()
    """

    def __init__(self):
        self._node = rclpy.create_node('mission_control')
        self._status = StatusMarkerPublisher(self._node)
        self._robots: list = []

        # Live capability registry, keyed by resolved action name: both
        # subscriptions live as long as the session, so a capability that
        # appears, restarts or dies at any point of the mission is seen.
        self._registry_lock = threading.Lock()
        self._registry: dict[str, CapabilityInfo] = {}
        self._last_heartbeat: dict[str, float] = {}
        self._capability_description_sub = self._node.create_subscription(
            CapabilityDescription, '/capabilities', self._on_capability_description,
            QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._capability_status_sub = self._node.create_subscription(
            CapabilityStatus, '/capabilities_status', self._on_capability_status,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = spin_in_background(self._executor)

    def _on_capability_description(self, msg: CapabilityDescription) -> None:
        with self._registry_lock:
            info = self._registry.setdefault(msg.action_name, CapabilityInfo())
            apply_description(info, msg)

    def _on_capability_status(self, msg: CapabilityStatus) -> None:
        with self._registry_lock:
            info = self._registry.setdefault(msg.action_name, CapabilityInfo())
            info.robot = msg.robot
            info.capability = msg.capability
            info.action_name = msg.action_name
            info.busy = msg.busy
            self._last_heartbeat[msg.action_name] = time.monotonic()

    def _alive(self, action_name: str, now: float) -> bool:
        last = self._last_heartbeat.get(action_name)
        return last is not None and now - last < ALIVE_TIMEOUT_SEC

    def capabilities_of(self, robot_name: str) -> list:
        """Every capability currently known for `robot_name`; `active` = recent heartbeat."""
        now = time.monotonic()
        result = []
        with self._registry_lock:
            for action_name, info in self._registry.items():
                if info.robot == robot_name:
                    snapshot = copy.copy(info)
                    snapshot.active = self._alive(action_name, now)
                    result.append(snapshot)
        return result

    def is_alive(self, robot_name: str, capability_type: str) -> bool:
        """Return whether a heartbeat was seen recently for this capability."""
        now = time.monotonic()
        with self._registry_lock:
            return any(
                info.robot == robot_name and info.capability == capability_type and
                self._alive(action_name, now)
                for action_name, info in self._registry.items())

    def add_robot(self, robot) -> None:
        """Add `robot` to this session, attaching it so it can be discovered and commanded."""
        if robot._session is not None:
            from .robot_handle import MissionManagerError
            raise MissionManagerError(
                f'FleetSession.add_robot("{robot.name}"): this robot is already attached '
                'to a session.')
        self._robots.append(robot)
        robot._attach(self)

    def remove_robot(self, robot) -> None:
        """Detach `robot` from this session, dropping its running goals."""
        if robot in self._robots:
            self._robots.remove(robot)
            robot._detach()

    @property
    def robots(self) -> list:
        """Every robot added so far, in add_robot() order."""
        return list(self._robots)

    def find_robot(self, name: str):
        """Look up a robot added so far by RobotHandle.name, or None if not found."""
        for robot in self._robots:
            if robot.name == name:
                return robot
        return None

    def discover_capabilities(self, window_sec: float = 2.5) -> None:
        """
        Wait `window_sec` so the live capability registry can fill up.

        Retained descriptions plus at least one 1 Hz heartbeat each. The
        registry keeps updating afterwards: this is a startup wait, not a
        snapshot.
        """
        self.spin_for(window_sec)

    def spin_some(self) -> None:
        """
        Give the background spin thread a time slice and check every robot's timeouts.

        The actual spinning happens continuously on the background
        thread started in __init__ -- this just gives it a slice of
        time to make progress and lets every added robot's in-flight
        run_capability() timeouts get checked.
        """
        time.sleep(0.01)
        for robot in self._robots:
            robot._check_timeouts()

    def spin_for(self, duration_sec: float) -> None:
        """Block for exactly `duration_sec`, still spinning underneath."""
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            self.spin_some()

    @property
    def node(self):
        """Underlying node, for anything not yet covered by RobotHandle."""
        return self._node

    @property
    def status_marker_publisher(self) -> StatusMarkerPublisher:
        """Status marker publisher RobotHandle.run_capability() publishes through."""
        return self._status

    def shutdown(self) -> None:
        """Stop the executor, join its background thread, and shut down the ROS context."""
        self._teardown()
        rclpy.shutdown()

    def _teardown(self) -> None:
        """
        Stop the background spin thread and destroy the node, without touching rclpy itself.

        Internal: unlike shutdown(), this leaves rclpy usable for the
        rest of the process -- Python has no deterministic destructor to
        call this automatically the way ~FleetSession() does in C++, so
        tests that construct several FleetSessions in one process
        (sharing one rclpy.init()/rclpy.shutdown() pair) call this
        directly between them instead of shutdown().
        """
        if self._spin_thread.is_alive():
            self._executor.shutdown()
            self._spin_thread.join()
        # Robots that outlive the session must not reach back into it.
        for robot in list(self._robots):
            robot._detach()
        self._robots.clear()
        self._executor.remove_node(self._node)
        self._node.destroy_node()
