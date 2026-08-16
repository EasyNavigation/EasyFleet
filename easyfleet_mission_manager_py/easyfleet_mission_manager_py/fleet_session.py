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

import time

from easyfleet_interfaces.msg import CapabilityStatus
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

from .capability_discovery import discover_capabilities as _discover_capabilities
from .mission_helpers import spin_in_background
from .status_markers import StatusMarkerPublisher


class FleetSession:
    """
    Everything talking to a fleet from a mission-control process actually needs.

    Regardless of what decides *when* to command which robot: the ROS
    node and background-spinning executor every RobotHandle rides on,
    one shared capability discovery pass, the registry of added robots,
    and the automatic status markers RobotHandle.run_capability()
    publishes.

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
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = spin_in_background(self._executor)
        self._status = StatusMarkerPublisher(self._node)
        self._robots: list = []

        # Persistent (unlike discover_capabilities()'s own temporary
        # subscription) so is_alive() reflects the *current* liveness of
        # every robot's capabilities for the whole mission, not just a
        # discovery-time snapshot -- dispatched to whichever attached
        # RobotHandle matches the message's robot field.
        self._capability_status_sub = self._node.create_subscription(
            CapabilityStatus, '/capabilities_status', self._on_capability_status,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def _on_capability_status(self, msg: CapabilityStatus) -> None:
        now = time.monotonic()
        robot = self.find_robot(msg.robot)
        if robot is not None:
            robot._note_heartbeat(msg.capability, now)

    def add_robot(self, robot) -> None:
        """Add `robot` to this session, attaching it so it can be discovered and commanded."""
        self._robots.append(robot)
        robot._attach(self)

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
        Run exactly one discovery scan and fan the results out to every added RobotHandle.

        Never a redundant per-robot re-scan.
        """
        all_capabilities = _discover_capabilities(self._node, window_sec)
        for robot in self._robots:
            this_robot = [info for info in all_capabilities if info.robot == robot.name]
            robot._set_capabilities(this_robot)

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
        self._executor.remove_node(self._node)
        self._node.destroy_node()
