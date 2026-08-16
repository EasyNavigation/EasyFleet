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

"""StatusMarkerPublisher, mirroring easyfleet_mission_manager's status_markers.hpp."""

import threading

from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


class StatusMarkerPublisher:
    """
    Publishes a floating TEXT_VIEW_FACING marker above each robot's own base_link.

    Shows a short, human-readable line of what that robot is currently
    doing -- so watching the mission in RViz alone is enough to follow
    along. One marker per robot, keyed by name: each set_status() call
    *replaces* that robot's previous text (same marker id).
    """

    def __init__(
        self, node, topic: str = 'mission_status_markers',
        height: float = 0.75, text_size: float = 0.25,
    ):
        self._node = node
        self._height = height
        self._text_size = text_size
        # Transient-local: a subscriber (RViz) that joins after the
        # mission has already started still needs to see every robot's
        # last known status immediately, not wait for its next
        # transition.
        self._pub = node.create_publisher(
            MarkerArray, topic,
            QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._lock = threading.Lock()
        self._texts: dict[str, str] = {}
        self._marker_ids: dict[str, int] = {}
        self._next_id = 0
        # set_status() is only called at mission phase transitions --
        # re-publishing every robot's current marker on a short timer
        # keeps every stamp recent, so RViz keeps resolving
        # "<robot>/base_link" and moving the text with the robot,
        # independent of how often the *text* itself actually changes.
        self._refresh_timer = node.create_timer(0.2, self._publish_all)

    def set_status(self, robot: str, text: str) -> None:
        """Set `robot`'s current status text and publish it immediately."""
        with self._lock:
            self._texts[robot] = text
            self._marker_id(robot)
        self._publish_all()

    def _marker_id(self, robot: str) -> int:
        # Requires self._lock to already be held by the caller.
        marker_id = self._marker_ids.get(robot)
        if marker_id is None:
            marker_id = self._next_id
            self._next_id += 1
            self._marker_ids[robot] = marker_id
        return marker_id

    def _publish_all(self) -> None:
        array = MarkerArray()
        stamp = self._node.get_clock().now().to_msg()
        with self._lock:
            items = list(self._texts.items())
            marker_ids = dict(self._marker_ids)

        for robot, text in items:
            marker = Marker()
            marker.header.frame_id = f'{robot}/base_link'
            marker.header.stamp = stamp
            marker.ns = 'mission_status'
            marker.id = marker_ids[robot]
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            # Re-transform against the *latest* available transform on
            # every render frame instead of an exact-header.stamp
            # lookup, which is racy under sim time.
            marker.frame_locked = True
            marker.pose.position.z = self._height
            marker.pose.orientation.w = 1.0
            marker.scale.z = self._text_size
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = text
            # Slightly longer than the refresh period, so a marker never
            # visibly blinks out between two refresh ticks.
            marker.lifetime = Duration(seconds=0.5).to_msg()
            array.markers.append(marker)

        if array.markers:
            self._pub.publish(array)
