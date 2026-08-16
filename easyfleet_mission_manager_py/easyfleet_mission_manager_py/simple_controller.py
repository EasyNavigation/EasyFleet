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

"""SimpleController, mirroring easyfleet_mission_manager's simple_controller.hpp."""

from .fleet_session import FleetSession


class SimpleController:
    """
    The plain, manual controller: a mission script decides everything by hand.

    SimpleController just hosts the FleetSession that makes talking to
    those robots possible -- there is no behavior here beyond the
    session's own. This is deliberate: it's meant to be the reference
    example of "a controller with zero decision-making logic of its
    own", also useful as a template for writing a different one.

    Usage:
        controller = SimpleController()

        robot_1 = RobotHandle('robot_1')
        robot_2 = RobotHandle('robot_2')
        controller.add_robot(robot_1)
        controller.add_robot(robot_2)

        controller.discover_capabilities()
        # ... robot_1.run_capability(...), controller.spin_some()/spin_for(...) ...

        controller.shutdown()
    """

    def __init__(self):
        self._session = FleetSession()

    def add_robot(self, robot) -> None:
        self._session.add_robot(robot)

    @property
    def robots(self) -> list:
        return self._session.robots

    def find_robot(self, name: str):
        return self._session.find_robot(name)

    def discover_capabilities(self, window_sec: float = 2.5) -> None:
        self._session.discover_capabilities(window_sec)

    def spin_some(self) -> None:
        self._session.spin_some()

    def spin_for(self, duration_sec: float) -> None:
        self._session.spin_for(duration_sec)

    @property
    def node(self):
        return self._session.node

    def shutdown(self) -> None:
        self._session.shutdown()
