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

"""
Test for FleetSession.shutdown(), mirroring test_fleet_session_shutdown.cpp.

FleetSession.shutdown() calls rclpy.shutdown(), invalidating the ROS
context for the rest of the process -- kept to its own local
rclpy.init()/rclpy.shutdown() pair (this module's only interaction with
rclpy) rather than sharing a fixture with the other test modules here,
so it can't race with them when pytest runs every test file in one
process.
"""

from easyfleet_mission_manager_py.fleet_session import FleetSession
import rclpy


def test_shutdown_stops_the_context():
    rclpy.init()
    session = FleetSession()
    assert rclpy.ok() is True

    session.shutdown()

    assert rclpy.ok() is False
