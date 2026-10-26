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

"""RobotStatusPublisher, mirroring easyfleet_core's robot_status_publisher.hpp/.cpp."""

import threading

from easyfleet_interfaces.msg import RobotStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


class RobotStatusPublisher:
    """
    Publishes the aggregated status of a robot process on /robot_status.

    Parameters, declared on `node`:
     - `robot_status.rate_hz` (1.0): publishing rate; 0 disables it.
     - `robot_status.pose_topic` (empty): a PoseWithCovarianceStamped topic
       (e.g. "amcl_pose") whose last pose is reported.
     - `robot_status.battery_topic` (empty): a BatteryState topic whose last
       message is reported.

    The robot is `node`'s namespace, and the capabilities come from
    `collect()` (e.g. `lambda: [c.get_status() for c in capabilities]`).
    """

    def __init__(self, node, collect):
        self._node = node
        self._collect = collect
        namespace = node.get_namespace()
        self._robot = namespace[1:] if namespace.startswith('/') else namespace
        rate_hz = node.declare_parameter('robot_status.rate_hz', 1.0).value
        pose_topic = node.declare_parameter('robot_status.pose_topic', '').value
        battery_topic = node.declare_parameter('robot_status.battery_topic', '').value

        self._lock = threading.Lock()
        self._pose = None
        self._battery = None
        self._pub = node.create_publisher(
            RobotStatus, '/robot_status',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._pose_sub = None
        self._battery_sub = None
        if pose_topic:
            self._pose_sub = node.create_subscription(
                PoseWithCovarianceStamped, pose_topic, self._on_pose,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        if battery_topic:
            self._battery_sub = node.create_subscription(
                BatteryState, battery_topic, self._on_battery, qos_profile_sensor_data)
        self._timer = node.create_timer(1.0 / rate_hz, self.publish) if rate_hz > 0.0 else None

    def _on_pose(self, msg) -> None:
        with self._lock:
            self._pose = msg

    def _on_battery(self, msg) -> None:
        with self._lock:
            self._battery = msg

    def make_status(self) -> RobotStatus:
        """Return the message that would be published now."""
        msg = RobotStatus()
        msg.robot = self._robot
        msg.stamp = self._node.get_clock().now().to_msg()
        if self._collect is not None:
            msg.capabilities = list(self._collect())
        with self._lock:
            if self._pose is not None:
                msg.has_pose = True
                msg.pose = self._pose
            if self._battery is not None:
                msg.has_battery = True
                msg.battery = self._battery
        return msg

    def publish(self) -> None:
        """Publish one message now."""
        self._pub.publish(self.make_status())
