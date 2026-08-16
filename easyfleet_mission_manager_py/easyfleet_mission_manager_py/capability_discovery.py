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

"""discover_capabilities(), mirroring easyfleet_mission_manager's capability_discovery.hpp/.cpp."""

import json
import threading
import time

from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .capability_info import CapabilityInfo


def discover_capabilities(node, window_sec: float = 2.5) -> list[CapabilityInfo]:
    """
    Listen on /capabilities and /capabilities_status for `window_sec`.

    Returns one CapabilityInfo per distinct `action_name` seen on either
    topic -- this is the right key because two robots can offer the same
    `capability` (e.g. "navigation") under different namespaces.
    `node` must already be part of an executor that is being spun on
    another thread: this function only subscribes and waits, it does not
    spin.
    """
    lock = threading.Lock()
    by_action_name: dict[str, CapabilityInfo] = {}

    def on_description(msg: CapabilityDescription) -> None:
        with lock:
            info = by_action_name.setdefault(msg.action_name, CapabilityInfo())
            info.robot = msg.robot
            info.capability = msg.capability
            info.action_name = msg.action_name
            info.description_json_raw = msg.description_json
            try:
                parsed = json.loads(msg.description_json)
                info.description_json = parsed if isinstance(parsed, dict) else {}
                info.description_json_valid = True
            except (ValueError, TypeError):
                info.description_json = {}
                info.description_json_valid = False

    def on_status(msg: CapabilityStatus) -> None:
        with lock:
            info = by_action_name.setdefault(msg.action_name, CapabilityInfo())
            info.robot = msg.robot
            info.capability = msg.capability
            info.action_name = msg.action_name
            info.active = True
            info.busy = msg.busy

    capabilities_sub = node.create_subscription(
        CapabilityDescription, '/capabilities', on_description,
        QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
    status_sub = node.create_subscription(
        CapabilityStatus, '/capabilities_status', on_status,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    time.sleep(window_sec)

    with lock:
        result = list(by_action_name.values())

    node.destroy_subscription(capabilities_sub)
    node.destroy_subscription(status_sub)

    return result
