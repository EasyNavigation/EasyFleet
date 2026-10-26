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
Lifecycle node that advertises a single action as an EasyFleet "capability".

Python mirror of easyfleet_core::Capability<ActionServerT> (see that
class's own docstring in easyfleet_core/capability.hpp for the full design
rationale, which this follows exactly): `Capability` is a
`rclpy.lifecycle.LifecycleNode` that:
 - owns an instance of an `ActionServerBase` subclass (constructed as
   `action_server_factory(self, capability_name)`);
 - on `on_activate`, reads the file named by the `capabilities_file`
   string parameter and publishes it, together with its own runtime
   identity (namespace and fully-qualified action name), as an
   `easyfleet_interfaces/CapabilityDescription` on the reliable,
   transient-local `/capabilities` topic, so late subscribers still get it;
 - while active, publishes an `easyfleet_interfaces/CapabilityStatus`
   heartbeat once a second on `/capabilities_status`;
 - keeps its recent orders (`<capability>.order_history`, 20 by default),
   queryable on the `~/get_order` service (e.g. /robot1/navigation/get_order);
 - with `<capability>.link_timeout` > 0 s, watches `/fleet_heartbeat` and,
   when it is not heard for that long and `<capability>.on_link_lost` is
   "cancel", stops the running goal ("continue", the default, only reports it).

Unlike the C++ template (`Capability<ActionServerT>`, one class per action
server type), a Python capability author instead passes the action server
*class* itself as a plain constructor argument -- there is no equivalent
need for a `CapabilityNodeBase`/`CapabilityFactory`/pluginlib layer here:
`rclpy.lifecycle.LifecycleNode` already exposes `trigger_configure()`/
`trigger_activate()`/`trigger_deactivate()`/`trigger_cleanup()`/
`trigger_shutdown()` directly (Python has no diamond-inheritance name
collision forcing the C++ side's `*_node()` renaming), and a Python
capability backend is loaded the ordinary way -- importing the module and
constructing the class -- rather than via a compiled plugin lookup.
"""

import time

from action_msgs.msg import GoalStatusArray
from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus, FleetHeartbeat
from easyfleet_interfaces.srv import GetOrder
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .action_server_base import _sanitize_parameter_name
from .link_watchdog import LinkWatchdog
from .order_history import OrderHistory

_CAPABILITIES_FILE_PARAM = 'capabilities_file'


def _strip_leading_slash(namespace: str) -> str:
    """`namespace` without its leading '/', so "/" becomes "" and "/robot1" becomes "robot1"."""
    return namespace[1:] if namespace.startswith('/') else namespace


class Capability(LifecycleNode):
    """A lifecycle node that advertises one action server as an EasyFleet capability."""

    def __init__(self, capability_name: str, action_server_factory, **kwargs):
        """
        Construct the capability node.

        :param capability_name: Used as the node name, the action name of
            the contained action server, and the `capability` identity
            field.
        :param action_server_factory: Callable `(node, action_name) ->
            ActionServerBase` building the action server this capability
            advertises -- typically the `ActionServerBase` subclass itself.
        :param kwargs: Forwarded to the underlying `LifecycleNode`.
        """
        super().__init__(capability_name, **kwargs)
        self._capability_name = capability_name
        self.declare_parameter(_CAPABILITIES_FILE_PARAM, '')
        prefix = _sanitize_parameter_name(capability_name)
        self._order_history_param = prefix + '.order_history'
        self._link_timeout_param = prefix + '.link_timeout'
        self._on_link_lost_param = prefix + '.on_link_lost'
        self.declare_parameter(self._order_history_param, 20)
        self.declare_parameter(self._link_timeout_param, 0.0)
        self.declare_parameter(self._on_link_lost_param, 'continue')
        self._order_history = OrderHistory(self.get_parameter(self._order_history_param).value)
        self._link_watchdog = LinkWatchdog()
        self._cancel_on_link_lost = False

        self._action_server = action_server_factory(self, capability_name)
        # Goals are only accepted while the capability is ACTIVE.
        self._action_server.disable_goals()

        self._robot_name = _strip_leading_slash(self.get_namespace())
        self._resolved_action_name = self.resolve_topic_name(
            self._action_server.get_action_name())

        self.get_logger().info(
            f"Starting capability '{capability_name}' on robot "
            f"'{self._robot_name or '-'}' (action: {self._resolved_action_name}).")

        self._capabilities_pub = None
        self._status_pub = None
        self._heartbeat_timer = None
        self._get_order_srv = None
        self._fleet_heartbeat_sub = None
        self._link_timer = None

        self._action_server.set_goal_accepted_callback(self._on_goal_accepted)
        # The action server's own status topic: the states of every goal it holds.
        self._goal_status_sub = self.create_subscription(
            GoalStatusArray, self._resolved_action_name + '/_action/status',
            lambda msg: self._order_history.update(msg, self.get_clock().now().to_msg()),
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def get_action_server(self):
        """Return the `ActionServerBase` backing this capability."""
        return self._action_server

    def get_capability_name(self) -> str:
        """Return the `capability` identity field published on /capabilities(_status)."""
        return self._capability_name

    def get_robot_name(self) -> str:
        """Return this node's namespace, without the leading '/' (empty if none)."""
        return self._robot_name

    def get_resolved_action_name(self) -> str:
        """Return the fully-qualified name of the action this capability exposes."""
        return self._resolved_action_name

    @property
    def order_history(self) -> OrderHistory:
        """Return the recent orders of this capability."""
        return self._order_history

    def get_status(self) -> CapabilityStatus:
        """Return this capability's status, as in its heartbeat."""
        msg = CapabilityStatus(
            robot=self._robot_name,
            capability=self._capability_name,
            action_name=self._resolved_action_name,
            busy=self._action_server.is_active(),
            link_state=self._link_watchdog.state)
        order = self._order_history.latest()
        if order is not None:
            msg.order_id = order.order_id
            msg.order_state = order.state
        return msg

    def _on_goal_accepted(self, goal_id, order_id: str) -> None:
        order = self._order_history.accepted(
            goal_id, order_id, self._resolved_action_name, self.get_clock().now().to_msg())
        self.get_logger().info(f"Capability '{self._capability_name}': order '{order}' accepted.")

    def _on_get_order(self, request, response):
        order = self._order_history.find(request.order_id)
        if order is not None:
            response.found = True
            response.order = order
        return response

    def on_configure(self, state) -> TransitionCallbackReturn:
        """Create the (inactive) publishers and the get_order service."""
        del state
        on_link_lost = self.get_parameter(self._on_link_lost_param).value
        if on_link_lost not in ('continue', 'cancel'):
            self.get_logger().error(
                f"Capability '{self._capability_name}': '{self._on_link_lost_param}' must be "
                f'"continue" or "cancel", not "{on_link_lost}".')
            return TransitionCallbackReturn.FAILURE
        self._cancel_on_link_lost = on_link_lost == 'cancel'
        self._capabilities_pub = self.create_lifecycle_publisher(
            CapabilityDescription, '/capabilities',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_pub = self.create_lifecycle_publisher(
            CapabilityStatus, '/capabilities_status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._get_order_srv = self.create_service(GetOrder, '~/get_order', self._on_get_order)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        """Publish the capability description and start the status heartbeat."""
        file_path = self.get_parameter(_CAPABILITIES_FILE_PARAM).value
        if not file_path:
            self.get_logger().error(
                f"Capability '{self._capability_name}': the '{_CAPABILITIES_FILE_PARAM}' "
                'parameter must be set to a capabilities JSON file path.')
            return TransitionCallbackReturn.FAILURE

        try:
            with open(file_path) as f:
                contents = f.read()
        except OSError as e:
            self.get_logger().error(
                f"Capability '{self._capability_name}': could not open capabilities file "
                f"'{file_path}': {e}.")
            return TransitionCallbackReturn.FAILURE

        # Activates capabilities_pub_/status_pub_ so publish() below actually
        # sends, only once the file is known to be readable; a subclass
        # overriding on_activate must call this or the publishers stay muted.
        super().on_activate(state)

        self._capabilities_pub.publish(CapabilityDescription(
            robot=self._robot_name,
            capability=self._capability_name,
            action_name=self._resolved_action_name,
            description_json=contents))

        # Wall clock, as in C++: a paused simulation must not make the
        # capability look dead.
        self._heartbeat_timer = self.create_timer(
            1.0, self._publish_heartbeat, clock=Clock(clock_type=ClockType.STEADY_TIME))

        link_timeout = float(self.get_parameter(self._link_timeout_param).value)
        self._link_watchdog.reset(link_timeout)
        if self._link_watchdog.monitored:
            self._fleet_heartbeat_sub = self.create_subscription(
                FleetHeartbeat, '/fleet_heartbeat',
                lambda msg: self._link_watchdog.heard(time.monotonic()),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            # A few checks per timeout, at most one a second.
            self._link_timer = self.create_timer(
                min(max(link_timeout / 4.0, 0.01), 1.0), self._check_link,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.get_logger().info(
                f"Capability '{self._capability_name}': watching /fleet_heartbeat (timeout "
                f'{link_timeout:.2f} s, on loss: '
                f"{'cancel' if self._cancel_on_link_lost else 'continue'}).")
        self._action_server.enable_goals()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        """Stop the running goal, reject new ones and stop the status heartbeat."""
        self._action_server.disable_goals()
        self._stop_heartbeat()
        super().on_deactivate(state)
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        """Release the resources acquired in `on_configure()`."""
        del state
        self._stop_heartbeat()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        """Stop the running goal and the status heartbeat if still running."""
        del state
        self._action_server.disable_goals()
        self._stop_heartbeat()
        self._destroy_publishers()
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self) -> None:
        """Stop the contained action server's worker thread, then destroy the node itself."""
        self._action_server.set_goal_accepted_callback(None)
        self._action_server.destroy()
        super().destroy_node()

    def _publish_heartbeat(self) -> None:
        self._status_pub.publish(self.get_status())

    def _check_link(self) -> None:
        before = self._link_watchdog.state
        now = self._link_watchdog.update(time.monotonic())
        if now == before:
            return
        if now == CapabilityStatus.LINK_LOST:
            stopping = self._cancel_on_link_lost and self._action_server.is_active()
            self.get_logger().warning(
                f"Capability '{self._capability_name}': link to the fleet lost "
                f"(no /fleet_heartbeat){', stopping the running goal' if stopping else ''}.")
            if self._cancel_on_link_lost:
                self._action_server.halt_active_goal()
        elif now == CapabilityStatus.LINK_OK and before == CapabilityStatus.LINK_LOST:
            self.get_logger().info(
                f"Capability '{self._capability_name}': link to the fleet recovered.")

    def _stop_heartbeat(self) -> None:
        if self._heartbeat_timer is not None:
            self._heartbeat_timer.cancel()
            self.destroy_timer(self._heartbeat_timer)
            self._heartbeat_timer = None
        if self._link_timer is not None:
            self._link_timer.cancel()
            self.destroy_timer(self._link_timer)
            self._link_timer = None
        if self._fleet_heartbeat_sub is not None:
            self.destroy_subscription(self._fleet_heartbeat_sub)
            self._fleet_heartbeat_sub = None

    def _destroy_publishers(self) -> None:
        if self._get_order_srv is not None:
            self.destroy_service(self._get_order_srv)
            self._get_order_srv = None
        if self._capabilities_pub is not None:
            self.destroy_lifecycle_publisher(self._capabilities_pub)
            self._capabilities_pub = None
        if self._status_pub is not None:
            self.destroy_lifecycle_publisher(self._status_pub)
            self._status_pub = None


def spin_capability(node: Capability) -> None:
    """
    Configure, activate and spin `node` until interrupted, then tear it down.

    A capability author's `main()` is typically just this one call plus
    the `Capability` construction: hides the
    configure/activate/.../spin/deactivate/cleanup lifecycle dance a
    standalone capability process needs to go through, mirroring how a
    C++ Deployment drives a capability's lifecycle on that side. Requires
    a `MultiThreadedExecutor` -- see `ActionServerBase`'s module docstring
    for why a plain `SingleThreadedExecutor` will not do.

    :param node: A configured-but-not-yet-activated `Capability` instance.
    :raises RuntimeError: If configuring or activating `node` fails --
        e.g. its `capabilities_file` parameter is unset or points to a
        file that cannot be read.
    """
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    configured = active = False
    try:
        configured = node.trigger_configure() == TransitionCallbackReturn.SUCCESS
        if not configured:
            raise RuntimeError(f"Capability '{node.get_capability_name()}': configure failed.")
        active = node.trigger_activate() == TransitionCallbackReturn.SUCCESS
        if not active:
            raise RuntimeError(f"Capability '{node.get_capability_name()}': activate failed.")
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Only the transitions valid from the state actually reached, so a
        # failed configure/activate isn't hidden behind a transition error.
        # After Ctrl-C the context may already be shut down, and then no
        # transition can run at all.
        if node.context.ok():
            if active:
                node.trigger_deactivate()
            if configured:
                node.trigger_cleanup()
        executor.shutdown()
        node.destroy_node()
