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
   heartbeat once a second on `/capabilities_status`.

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

from easyfleet_interfaces.msg import CapabilityDescription, CapabilityStatus
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

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

    def on_configure(self, state) -> TransitionCallbackReturn:
        """Create the (inactive) /capabilities and /capabilities_status publishers."""
        del state
        self._capabilities_pub = self.create_lifecycle_publisher(
            CapabilityDescription, '/capabilities',
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._status_pub = self.create_lifecycle_publisher(
            CapabilityStatus, '/capabilities_status',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
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
        self._action_server.destroy()
        super().destroy_node()

    def _publish_heartbeat(self) -> None:
        self._status_pub.publish(CapabilityStatus(
            robot=self._robot_name,
            capability=self._capability_name,
            action_name=self._resolved_action_name,
            busy=self._action_server.is_active()))

    def _stop_heartbeat(self) -> None:
        if self._heartbeat_timer is not None:
            self._heartbeat_timer.cancel()
            self.destroy_timer(self._heartbeat_timer)
            self._heartbeat_timer = None

    def _destroy_publishers(self) -> None:
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
