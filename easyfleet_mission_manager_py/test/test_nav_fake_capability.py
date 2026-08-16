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
Minimal fake navigation capability for exercising RobotHandle end to end.

Mirrors easyfleet_mission_manager's own test/test_nav_fake_capability.hpp
(TestNavActionServer/FakeRobotProcess): ignores goal content entirely
except to check for REJECT_WAYPOINT_ID, succeeds after
`mock_duration_sec` (0 by default -- fast tests), and honors
cancellation immediately.
"""

import threading
import time

from easyfleet_interfaces.action import Navigation
from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Magic waypoint id (parameters_json's goal_id) that makes the fake
# server reject the goal outright -- the only way this fake ever
# produces a genuine REJECTED (as opposed to CANCELED) outcome.
REJECT_WAYPOINT_ID = '__reject__'


class FakeNavCapabilityProcess:
    """
    Brings up one fake navigation action server, namespaced under `robot_name`.

    Spun on its own executor/thread -- simulating a robot's own,
    independent process the way discover_capabilities()/RobotHandle
    actually talk to one over ROS, not a shortcut into the same process.
    """

    def __init__(self, robot_name: str, mock_duration_sec: float = 0.0):
        self._robot_name = robot_name
        self._mock_duration_sec = mock_duration_sec
        self._busy = False

        self._node = rclpy.create_node(
            'fake_nav_capability', namespace=f'/{robot_name}',
            start_parameter_services=False)

        action_name = f'/{robot_name}/navigation'
        self._action_name = action_name
        action_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self._node, Navigation, action_name,
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=action_group)

        capabilities_pub = self._node.create_publisher(
            CapabilityDescription, '/capabilities',
            QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        description = CapabilityDescription()
        description.robot = robot_name
        description.capability = 'navigation'
        description.action_name = action_name
        description.description_json = '{"display_name": "Fake Navigation"}'
        capabilities_pub.publish(description)

        self._status_pub = self._node.create_publisher(
            CapabilityStatus, '/capabilities_status',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # A distinct (default, mutually-exclusive) callback group from
        # the action server's own reentrant one, so the heartbeat keeps
        # ticking under a MultiThreadedExecutor even while a goal is
        # mid-execution.
        self._status_timer = self._node.create_timer(1.0, self._publish_heartbeat)

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        while not self._executor.is_spinning:
            time.sleep(0)

    def _publish_heartbeat(self) -> None:
        msg = CapabilityStatus()
        msg.robot = self._robot_name
        msg.capability = 'navigation'
        msg.action_name = self._action_name
        msg.busy = self._busy
        self._status_pub.publish(msg)

    def _on_goal(self, goal_request):
        if REJECT_WAYPOINT_ID in goal_request.parameters_json:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute(self, goal_handle):
        self._busy = True
        try:
            step = 0.02
            remaining = self._mock_duration_sec
            while remaining > 0.0:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return Navigation.Result()
                time.sleep(step)
                remaining -= step
            goal_handle.succeed()
            return Navigation.Result()
        finally:
            self._busy = False

    def shutdown(self) -> None:
        self._executor.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join()
        self._node.destroy_node()
