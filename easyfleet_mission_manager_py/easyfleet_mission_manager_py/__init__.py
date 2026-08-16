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
Python mirror of easyfleet_mission_manager.

FleetSession/RobotHandle/SimpleController for writing mission
controllers in Python instead of C++, talking the same wire protocol
(/capabilities, /capabilities_status, the easyfleet_interfaces
actions) -- a Python mission script and a C++ one are interchangeable
against the same fleet.
"""

from .capability_client import CapabilityClient
from .capability_client import Outcome as CapabilityOutcome
from .capability_client import Response as CapabilityResponse
from .capability_discovery import discover_capabilities
from .capability_info import CapabilityInfo, print_capability_info, print_capability_summary_line
from .capability_state import CapabilityState
from .capability_state import to_string as capability_state_to_string
from .fleet_session import FleetSession
from .mission_helpers import (
    find_robot_capability,
    make_manipulation_feedback_printer,
    make_manipulation_goal,
    make_navigation_feedback_printer,
    make_navigation_goal,
    make_perception_feedback_printer,
    make_perception_goal,
    Manipulation,
    Navigation,
    Perception,
    print_section,
    print_step,
    RUN_TIMEOUT_SEC,
    spin_in_background,
)
from .robot_handle import MissionManagerError, RobotHandle
from .simple_controller import SimpleController
from .status_markers import StatusMarkerPublisher
from .throttle import Throttle

__all__ = [
    'RUN_TIMEOUT_SEC',
    'CapabilityClient',
    'CapabilityInfo',
    'CapabilityOutcome',
    'CapabilityResponse',
    'CapabilityState',
    'FleetSession',
    'Manipulation',
    'MissionManagerError',
    'Navigation',
    'Perception',
    'RobotHandle',
    'SimpleController',
    'StatusMarkerPublisher',
    'Throttle',
    'capability_state_to_string',
    'discover_capabilities',
    'find_robot_capability',
    'make_manipulation_feedback_printer',
    'make_manipulation_goal',
    'make_navigation_feedback_printer',
    'make_navigation_goal',
    'make_perception_feedback_printer',
    'make_perception_goal',
    'print_capability_info',
    'print_capability_summary_line',
    'print_section',
    'print_step',
    'spin_in_background',
]
