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
Shared, ROS-facing model for EasyFleet monitoring.

Reused as-is by both the TUI (easyfleet_tools.tui.app) and the ros2cli
verbs (easyfleet_tools.cli.*), so message parsing/formatting lives in
exactly one place.
"""

import collections
import json
import time

from action_msgs.msg import GoalStatusArray
from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
from rcl_interfaces.msg import Log
import rclpy.qos
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_action, get_message
from visualization_msgs.msg import MarkerArray


# How long without a /capabilities_status heartbeat before a capability is
# considered gone -- mirrors easyfleet_mission_manager's own
# RobotHandle::kAliveTimeout (a few times the capability's own 1 Hz
# heartbeat period, so one or two dropped messages don't flip it).
ALIVE_TIMEOUT = 3.0

# Mirrors easyfleet_mission_manager::capability_info.cpp's status_label():
# INACTIVE (no recent heartbeat) / BUSY (heartbeat + goal executing) /
# IDLE (heartbeat + free).
STATUS_COLOR = {
    'INACTIVE': 'red',
    'BUSY': 'yellow',
    'IDLE': 'green',
}


class CapabilityInfo:
    """
    Everything known about one running capability instance.

    Gathered from /capabilities and /capabilities_status -- the Python
    mirror of easyfleet_mission_manager::CapabilityInfo.
    """

    def __init__(self, robot: str, capability: str, action_name: str):
        self.robot = robot
        self.capability = capability
        self.action_name = action_name
        self.description_json_raw = ''
        self.description_json: dict | None = None
        self.busy = False
        self.last_heartbeat: float | None = None

    @property
    def status(self) -> str:
        if self.last_heartbeat is None or \
                (time.monotonic() - self.last_heartbeat) > ALIVE_TIMEOUT:
            return 'INACTIVE'
        return 'BUSY' if self.busy else 'IDLE'

    @property
    def display_name(self) -> str:
        if isinstance(self.description_json, dict):
            return self.description_json.get('display_name', self.capability)
        return self.capability

    def set_description(self, description_json_raw: str) -> None:
        self.description_json_raw = description_json_raw
        try:
            parsed = json.loads(description_json_raw)
            self.description_json = parsed if isinstance(parsed, dict) else None
        except (ValueError, TypeError):
            self.description_json = None


class RobotInfo:
    """
    One discovered robot.

    Holds its capabilities (keyed by capability type) and its latest
    mission-status-marker text (the same line RViz shows floating above
    it).
    """

    def __init__(self, name: str):
        self.name = name
        self.capabilities: dict[str, CapabilityInfo] = {}
        self.status_text = ''

    @property
    def worst_status(self) -> str | None:
        """
        Return the single status that best summarizes this robot at a glance.

        BUSY if any capability is busy, else INACTIVE if any capability
        has gone quiet, else IDLE if it has at least one capability, else
        None (nothing known about it yet).
        """
        if not self.capabilities:
            return None
        statuses = {c.status for c in self.capabilities.values()}
        if 'BUSY' in statuses:
            return 'BUSY'
        if 'INACTIVE' in statuses:
            return 'INACTIVE'
        return 'IDLE'


def _robot_from_base_link_frame(frame_id: str) -> str:
    suffix = '/base_link'
    return frame_id[:-len(suffix)] if frame_id.endswith(suffix) else frame_id


class FleetDiscovery:
    """
    Build and keep a live model of the fleet from three topics.

    /capabilities (who exists), /capabilities_status (is it alive/busy),
    mission_status_markers (what is it currently doing, in the same words
    RViz shows). Usable both as a live, callback-driven model (TUI) and
    as a plain poll-after-spin_once snapshot (CLI verbs).
    """

    def __init__(self, node, on_change=None):
        self.node = node
        self.on_change = on_change
        self.robots: dict[str, RobotInfo] = {}

        self._capabilities_sub = node.create_subscription(
            CapabilityDescription, '/capabilities',
            self._on_capability_description,
            rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_sub = node.create_subscription(
            CapabilityStatus, '/capabilities_status',
            self._on_capability_status,
            rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE))
        self._markers_sub = node.create_subscription(
            MarkerArray, 'mission_status_markers',
            self._on_status_markers,
            rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))

    def _robot(self, name: str) -> RobotInfo:
        robot = self.robots.get(name)
        if robot is None:
            robot = RobotInfo(name)
            self.robots[name] = robot
        return robot

    def _on_capability_description(self, msg: CapabilityDescription) -> None:
        robot = self._robot(msg.robot)
        capability = robot.capabilities.get(msg.capability)
        if capability is None:
            capability = CapabilityInfo(msg.robot, msg.capability, msg.action_name)
            robot.capabilities[msg.capability] = capability
        capability.action_name = msg.action_name
        capability.set_description(msg.description_json)
        self._notify()

    def _on_capability_status(self, msg: CapabilityStatus) -> None:
        robot = self._robot(msg.robot)
        capability = robot.capabilities.get(msg.capability)
        if capability is None:
            capability = CapabilityInfo(msg.robot, msg.capability, msg.action_name)
            robot.capabilities[msg.capability] = capability
        capability.busy = msg.busy
        capability.last_heartbeat = time.monotonic()
        self._notify()

    def _on_status_markers(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            robot_name = _robot_from_base_link_frame(marker.header.frame_id)
            self._robot(robot_name).status_text = marker.text
        self._notify()

    def _notify(self) -> None:
        if self.on_change is not None:
            self.on_change()


# ---------- Generic per-action monitoring (status/feedback topics) ----------

# Mirrors action_msgs/msg/GoalStatus's STATUS_* constants.
GOAL_STATUS_MAP: dict[int, tuple[str, str]] = {
    0: ('UNKNOWN', 'white'),
    1: ('ACCEPTED', 'yellow'),
    2: ('EXECUTING', 'green'),
    3: ('CANCELING', 'yellow'),
    4: ('SUCCEEDED', 'green'),
    5: ('CANCELED', 'yellow'),
    6: ('ABORTED', 'red'),
}

# A goal that reached one of these will never change status again --
# this is the point at which its actual Result content (not just the
# status code) becomes available via the `_action/get_result` service.
_TERMINAL_GOAL_STATUSES = {4, 5, 6}


def status_msg2text(msg: GoalStatusArray) -> str:
    if not msg.status_list:
        return 'No goals seen yet.'
    # The action server only ever tracks one goal at a time in this
    # project (a new one preempts the last) -- the most recently
    # announced entry is what matters.
    entry = msg.status_list[-1]
    label, color = GOAL_STATUS_MAP.get(entry.status, (str(entry.status), 'white'))
    return f'Goal status: [{color}]{label}[/{color}]'


def generic_msg2text(msg) -> str:
    """
    Format any message/sub-message as YAML.

    Used for both an action's live Feedback and its final Result, whose
    concrete field names (e.g. Navigation's
    `current_pose`/`distance_remaining`, Perception's `detections_3d`)
    vary per action type and are never hardcoded here.
    """
    return message_to_yaml(msg).strip()


class GenericActionMonitor:
    """
    Read-only monitor for an arbitrary, already-running action goal.

    Subscribes to an action's own `<action_name>/_action/status` and
    `<action_name>/_action/feedback` topics -- without being the client
    that sent the goal -- so any in-flight goal (sent by a mission
    script, not by us) can be observed read-only. Once a goal reaches a
    terminal status, its actual Result content is fetched with a
    `<action_name>/_action/get_result` service call: the action protocol
    lets *any* client fetch a goal's result given its goal_id, not just
    the one that originally sent it. Every message/service type involved
    is resolved dynamically (like `ros2 topic echo`/`ros2 service call`
    do), so this works for any action type, not just the three this
    project ships today.
    """

    def __init__(self, node, action_name: str, on_status=None, on_feedback=None, on_result=None):
        self.node = node
        self.action_name = action_name
        self.on_status = on_status
        self.on_feedback = on_feedback
        self.on_result = on_result
        self.last_status_text = 'Goal status: [white]unknown[/white]'
        self.last_feedback_text = '(no feedback yet)'
        self.last_result_text = '(no result yet)'

        self._current_goal_id: bytes | None = None
        self._pending_result_goal_id = None
        self._pending_result_goal_id_bytes: bytes | None = None
        self._result_requested_for: bytes | None = None

        self._status_sub = node.create_subscription(
            GoalStatusArray, f'{action_name}/_action/status',
            self._on_status, rclpy.qos.qos_profile_action_status_default)
        self._feedback_sub = None
        self._get_result_client = None
        self._get_result_request_cls = None
        self._resolve_timer = node.create_timer(0.5, self._on_resolve_tick)
        self._on_resolve_tick()

    def _on_resolve_tick(self) -> None:
        self._try_resolve_feedback()
        self._try_resolve_get_result_client()
        self._maybe_request_result()

    def _try_resolve_feedback(self) -> None:
        if self._feedback_sub is not None:
            return
        topic = f'{self.action_name}/_action/feedback'
        for name, types in self.node.get_topic_names_and_types():
            if name != topic:
                continue
            for type_str in types:
                msg_class = get_message(type_str)
                if msg_class is None:
                    continue
                self._feedback_sub = self.node.create_subscription(
                    msg_class, topic, self._on_feedback, 10)
                return

    def _try_resolve_get_result_client(self) -> None:
        if self._get_result_client is not None:
            return
        service_name = f'{self.action_name}/_action/get_result'
        for name, types in self.node.get_service_names_and_types():
            if name != service_name:
                continue
            for type_str in types:
                # type_str is "<pkg>/action/<Name>_GetResult" --
                # rosidl_runtime_py.utilities.get_service() can't resolve
                # this: actions generate loose
                # <Name>_GetResult_Request/_Response classes, not a
                # single <Name>_GetResult class with nested
                # .Request/.Response the way a plain .srv-based service
                # (and get_service()) expects. Strip the suffix to get
                # the action's own type string instead, resolve *that*
                # with get_action(), and pull the properly-nested
                # service class from Action.Impl.GetResultService -- the
                # same type rclpy.action.ActionClient itself uses
                # internally for this exact service.
                if not type_str.endswith('_GetResult'):
                    continue
                action_cls = get_action(type_str[:-len('_GetResult')])
                if action_cls is None:
                    continue
                srv_class = action_cls.Impl.GetResultService
                self._get_result_client = self.node.create_client(srv_class, service_name)
                self._get_result_request_cls = srv_class.Request
                return

    def _on_status(self, msg: GoalStatusArray) -> None:
        self.last_status_text = status_msg2text(msg)
        if self.on_status is not None:
            self.on_status(self.last_status_text)
        if not msg.status_list:
            return

        entry = msg.status_list[-1]
        goal_id_bytes = bytes(entry.goal_info.goal_id.uuid)
        if goal_id_bytes != self._current_goal_id:
            # A different (newer) goal than the one this was last
            # tracking -- its own result, once any, replaces the
            # previous goal's.
            self._current_goal_id = goal_id_bytes
            self.last_result_text = '(no result yet)'

        if entry.status in _TERMINAL_GOAL_STATUSES:
            self._pending_result_goal_id = entry.goal_info.goal_id
            self._pending_result_goal_id_bytes = goal_id_bytes
            self._maybe_request_result()

    def _maybe_request_result(self) -> None:
        if self._pending_result_goal_id_bytes is None:
            return
        if self._result_requested_for == self._pending_result_goal_id_bytes:
            return
        if self._get_result_client is None or not self._get_result_client.service_is_ready():
            return  # retried on the next resolve tick

        self._result_requested_for = self._pending_result_goal_id_bytes
        request = self._get_result_request_cls()
        request.goal_id = self._pending_result_goal_id
        future = self._get_result_client.call_async(request)
        future.add_done_callback(
            lambda f, expected=self._pending_result_goal_id_bytes: self._on_result_response(
                f, expected))

    def _on_result_response(self, future, expected_goal_id_bytes: bytes) -> None:
        try:
            response = future.result()
        except Exception:
            if self._result_requested_for == expected_goal_id_bytes:
                self._result_requested_for = None  # let the next tick retry
            return
        if expected_goal_id_bytes != self._current_goal_id:
            return  # a newer goal has already superseded this one
        self.last_result_text = generic_msg2text(response.result)
        if self.on_result is not None:
            self.on_result(self.last_result_text)

    def _on_feedback(self, msg) -> None:
        self.last_feedback_text = generic_msg2text(msg.feedback)
        if self.on_feedback is not None:
            self.on_feedback(self.last_feedback_text)

    def destroy(self) -> None:
        self.node.destroy_timer(self._resolve_timer)
        self.node.destroy_subscription(self._status_sub)
        if self._feedback_sub is not None:
            self.node.destroy_subscription(self._feedback_sub)
        if self._get_result_client is not None:
            self.node.destroy_client(self._get_result_client)


# ---------- Fleet-wide /rosout monitoring ----------

# Per the TUI/CLI's own spec: debug=green, info=white, warning=orange,
# error=red. rcl_interfaces/msg/Log.FATAL (50) has no explicit color of
# its own in that spec -- treated as a more severe error, bold red.
LOG_LEVEL_MAP: dict[int, tuple[str, str]] = {
    Log.DEBUG: ('DEBUG', 'green'),
    Log.INFO: ('INFO', 'white'),
    Log.WARN: ('WARN', 'orange3'),
    Log.ERROR: ('ERROR', 'red'),
    Log.FATAL: ('FATAL', 'bold red'),
}


def log_msg2text(msg: Log) -> str:
    label, color = LOG_LEVEL_MAP.get(msg.level, (str(msg.level), 'white'))
    return f'[{color}][{label}] {msg.name}: {msg.msg}[/{color}]'


class RosoutMonitor:
    """
    One shared /rosout subscription, fanned out to whoever is interested.

    Per-logger-name callbacks (a capability's own log box) plus a
    bounded, fleet-wide ring buffer of WARN-and-above entries (an
    at-a-glance "what's gone wrong lately" feed, independent of which
    robot/capability is currently selected).
    """

    def __init__(self, node, warnings_maxlen: int = 200):
        self.node = node
        self.warnings: collections.deque = collections.deque(maxlen=warnings_maxlen)
        self._on_any: list = []
        self._by_logger_name: dict[str, list] = {}
        self._sub = node.create_subscription(Log, '/rosout', self._on_log, 10)

    def register(self, logger_name: str, callback) -> None:
        self._by_logger_name.setdefault(logger_name, []).append(callback)

    def unregister(self, logger_name: str, callback) -> None:
        callbacks = self._by_logger_name.get(logger_name)
        if callbacks and callback in callbacks:
            callbacks.remove(callback)

    def add_on_any(self, callback) -> None:
        """
        Register `callback` to run for every /rosout message.

        Regardless of logger name -- used by the fleet-wide Logs tab/verb
        and the Mission tab's "recent warnings" feed. Several callbacks
        can be registered at once (unlike `register()`, this has no
        matching `remove` -- every caller here lives for the whole
        app/process lifetime).
        """
        self._on_any.append(callback)

    def _on_log(self, msg: Log) -> None:
        text = log_msg2text(msg)
        if msg.level >= Log.WARN:
            self.warnings.append(text)
        for callback in self._by_logger_name.get(msg.name, []):
            callback(msg, text)
        for callback in self._on_any:
            callback(msg, text)
