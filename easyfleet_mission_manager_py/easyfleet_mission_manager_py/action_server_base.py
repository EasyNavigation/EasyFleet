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
Base class that hides the boilerplate of running a ROS 2 action server.

Python mirror of easyfleet_core::ActionServerBase<ActionT> (see that
class's own docstring in easyfleet_core/action_server_base.hpp for the
design this follows): a capability author attaches one of these to a node
and implements three hooks -- `on_goal_received()`, `on_execute()` and,
optionally, `on_cancel_requested()`/`on_preempted()`. Everything else --
wiring the rclpy `ActionServer`, transitioning the goal state machine, and
deciding whether an incoming goal is allowed to preempt the one currently
running -- is handled here.

Design difference from the C++ version: the C++ side runs every goal on
one dedicated worker thread it owns outright, and settles it by calling
the goal handle's succeed(result)/abort(result)/canceled(result)
directly. The rclpy equivalent of that argument -- passing the `Result`
straight to succeed()/abort()/canceled() -- silently races rclpy's own
action-server plumbing: `ServerGoalHandle.succeed(response)` completes
the get-result service's response future immediately, and then rclpy's
own `_execute_goal()` tries to complete that same, already-completed
future again with whatever the execute_callback *returns* -- corrupting
the result and crashing the client at the C message-conversion layer
(confirmed empirically while building this class). So here,
succeed()/abort()/canceled() are always called with **no argument**, and
`on_execute()` communicates the actual `Result` back by returning it
instead -- see its own docstring. `on_execute()` runs as the actual
`execute_callback` rclpy schedules for each accepted goal, on a
`ReentrantCallbackGroup` so multiple goals' callbacks (goal/cancel/
execute) can run concurrently -- required for preemption to work at all,
since a `MutuallyExclusive` group (the rclpy default) would block a new
goal's execute_callback until the preempted one's finishes. **The
hosting node must be spun with a `rclpy.executors.MultiThreadedExecutor`**
(a `SingleThreadedExecutor` can't run two callbacks -- e.g. a long
`on_execute()` and the next goal's acceptance -- at once regardless of
callback group).

Preemption is governed by a boolean ROS 2 parameter named
"<action_name>.allow_preemption" (with '/' in the action name mapped to
'.'), so it can be set at startup and changed at runtime. Conceptually
only one goal is active at a time:
 - If preemption is disallowed and a goal is active, new goals are
   rejected outright.
 - If preemption is allowed, a new goal is accepted immediately and
   `is_preempt_requested()` starts returning True *for the preempted
   goal's own `on_execute()` call* (tracked per goal, via the calling
   thread -- see `is_preempt_requested()`). A long-running `on_execute()`
   implementation is expected to poll it (and `is_shutdown_requested()`)
   and return promptly, having settled the goal handle. As in C++, the new
   goal's `on_execute()` only starts once the preempted one has returned,
   and a goal still waiting to start is aborted if an even newer one
   arrives.

Goals are only accepted while goals are enabled (the default).
`disable_goals()` rejects new goals, aborts waiting ones and waits for the
running one to return; `Capability` uses it to follow its lifecycle state.
"""

import threading

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


def _robot_label(namespace: str) -> str:
    """`namespace` without its leading '/', or "-" if that would be empty."""
    stripped = namespace[1:] if namespace.startswith('/') else namespace
    return stripped or '-'


def _sanitize_parameter_name(action_name: str) -> str:
    """`action_name` with leading '/'s stripped and remaining '/'s turned into '.'."""
    name = action_name.lstrip('/').replace('/', '.')
    return name or 'action'


class ActionServerBase:
    """Hides rclpy action-server boilerplate behind on_goal_received()/on_execute()."""

    def __init__(
            self, node, action_type, action_name: str,
            default_allow_preemption: bool = True):
        """
        Construct the action server, attached to `node` under `action_name`.

        :param node: Node (or lifecycle node) that will host the action
            server; must be spun with a `MultiThreadedExecutor` (see the
            module docstring). Only used here to read/declare parameters
            and to construct the underlying `rclpy.action.ActionServer` --
            not stored beyond that, so it must outlive this object.
        :param action_type: The `rclpy` action type (e.g.
            `easyfleet_interfaces.action.Perception`).
        :param action_name: Name under which the action is advertised.
        :param default_allow_preemption: Initial value of the preemption
            parameter, used only if it has not already been declared/set
            (e.g. from a YAML params file or the command line).
        """
        self._node = node
        self._action_type = action_type
        self._action_name = action_name
        self._robot_name = _robot_label(node.get_namespace())

        self._param_name = _sanitize_parameter_name(action_name) + '.allow_preemption'
        if not node.has_parameter(self._param_name):
            node.declare_parameter(self._param_name, default_allow_preemption)
        self._allow_preemption = bool(node.get_parameter(self._param_name).value)
        node.add_post_set_parameters_callback(self._on_parameters_set)

        self._lock = threading.Lock()
        # Notified whenever a goal stops running or goals are disabled.
        self._idle = threading.Condition(self._lock)
        self._shutting_down = False
        self._halt_requested = False
        self._goals_enabled = True
        # Keyed by id(goal_handle): ServerGoalHandle isn't hashable. Goals
        # accepted whose on_execute() hasn't started yet, and the one whose
        # on_execute() is running (at most one).
        self._waiting_handles = {}
        self._active_handles = {}
        # Which goal_handle "is_preempt_requested()" is answering about,
        # scoped per-thread since `_dispatch_execute()` may run
        # concurrently on more than one executor worker thread at once
        # (a just-preempted goal winding down while its successor has
        # already started).
        self._thread_local = threading.local()

        self._server = ActionServer(
            node, action_type, action_name,
            execute_callback=self._dispatch_execute,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            handle_accepted_callback=self._handle_accepted,
            callback_group=ReentrantCallbackGroup())

    def destroy(self) -> None:
        """Disable goals, wait for the running one and destroy the server; idempotent."""
        with self._lock:
            if self._shutting_down:
                return
        self.disable_goals()
        with self._lock:
            self._shutting_down = True
        self._node.remove_post_set_parameters_callback(self._on_parameters_set)
        self._server.destroy()

    def enable_goals(self) -> None:
        """Accept new goals again after `disable_goals()` (not after `destroy()`)."""
        with self._lock:
            if not self._shutting_down:
                self._goals_enabled = True

    def disable_goals(self) -> None:
        """
        Reject new goals, abort waiting ones and wait for the running one to return.

        `is_shutdown_requested()` is True while waiting. Must not be called
        from `on_execute()`.
        """
        with self._idle:
            self._goals_enabled = False
            if not self._waiting_handles and not self._active_handles:
                return
            self._halt_requested = True
            self._idle.notify_all()
            while self._waiting_handles or self._active_handles:
                self._idle.wait(0.05)
            self._halt_requested = False

    def goals_enabled(self) -> bool:
        """Return whether new goals are currently accepted."""
        with self._lock:
            return self._goals_enabled

    def get_action_name(self) -> str:
        """Return the name of the action served by this instance."""
        return self._action_name

    def is_active(self) -> bool:
        """Return whether a goal is currently executing."""
        with self._lock:
            return bool(self._active_handles)

    def is_preemptable(self) -> bool:
        """Return the current value of the "allow_preemption" parameter for this action."""
        return self._allow_preemption

    def is_preempt_requested(self) -> bool:
        """
        Return whether the calling `on_execute()`'s own goal has been preempted.

        Meaningful only when called from within `on_execute()` (from the
        thread that call is running on); returns False otherwise.
        """
        goal_handle = getattr(self._thread_local, 'goal_handle', None)
        if goal_handle is None:
            return False
        return getattr(goal_handle, '_easyfleet_preempted', False)

    def is_shutdown_requested(self) -> bool:
        """Return whether the running goal must stop: goals are being disabled or destroyed."""
        with self._lock:
            return self._shutting_down or self._halt_requested

    @property
    def logger(self):
        """Return the logger of the node hosting this action server."""
        return self._node.get_logger()

    # -- Hooks for subclasses --

    def on_goal_received(self, goal) -> bool:
        """
        Validate an incoming goal.

        Preemption bookkeeping is applied automatically after this returns
        True, so implementations only need to worry about whether the
        *content* of the goal is valid. Must be overridden.

        :param goal: Goal content to validate.
        :return: True to accept, False to reject.
        """
        raise NotImplementedError

    def on_execute(self, goal_handle):
        """
        Run an accepted goal to completion. Must be overridden.

        Must leave `goal_handle` in a terminal state -- `succeed()`,
        `abort()` or `canceled()`, called with **no argument** -- before
        returning, and must separately `return` the actual `Result`
        message: rclpy's own action-server plumbing uses the
        execute_callback's *return value* to answer the client's
        get_result request, not whatever (if anything) was passed to
        succeed()/abort()/canceled() (which would race it -- see the
        module docstring). Long-running implementations should
        periodically check `is_preempt_requested()`,
        `goal_handle.is_cancel_requested` and `is_shutdown_requested()`
        and return promptly when any of them is True. Only settle with
        `canceled()` when `goal_handle.is_cancel_requested` is True (a
        client asked for it); for preemption or shutdown, use `abort()`
        instead.

        :param goal_handle: Handle of the accepted goal to run to completion.
        :return: The action's `Result` message to send back to the client.
        """
        raise NotImplementedError

    def on_cancel_requested(self, goal_handle) -> CancelResponse:
        """
        Decide whether a cancel request should be accepted. Unrelated to preemption.

        Defaults to always accepting.

        :param goal_handle: Handle of the goal a cancellation was requested for.
        :return: Whether the cancel request is accepted.
        """
        del goal_handle
        return CancelResponse.ACCEPT

    def on_preempted(self, preempted_goal_handle) -> None:
        """
        Notify that a new goal is about to preempt the currently running one.

        Optional hook, called from the accepting callback (not from
        `preempted_goal_handle`'s own `on_execute()` thread). Default
        implementation does nothing.

        :param preempted_goal_handle: Handle of the goal being preempted.
        """
        del preempted_goal_handle

    # -- rclpy_action wiring --

    def _handle_goal(self, goal):
        with self._lock:
            if not self._goals_enabled:
                self.logger.warning(
                    f"Action '{self._action_name}': goal rejected, goals are disabled "
                    '(capability not active).')
                return GoalResponse.REJECT
        try:
            accepted = self.on_goal_received(goal)
        except Exception as e:  # A capability bug must not crash the executor.
            self.logger.error(
                f"Action '{self._action_name}': exception raised from on_goal_received(): {e}")
            return GoalResponse.REJECT
        if not accepted:
            return GoalResponse.REJECT
        with self._lock:
            if not self._goals_enabled:
                return GoalResponse.REJECT
            # A goal waiting to start counts as busy too: two goals arriving
            # back to back must not both be accepted when preemption is off.
            busy = self._active_handles or self._waiting_handles
            if busy and not self._allow_preemption:
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle):
        try:
            return self.on_cancel_requested(goal_handle)
        except Exception as e:
            self.logger.error(
                f"Action '{self._action_name}': exception raised from on_cancel_requested(): {e}")
            return CancelResponse.REJECT

    def _handle_accepted(self, goal_handle) -> None:
        with self._lock:
            preempted = list(self._active_handles.values()) + list(self._waiting_handles.values())
            self._waiting_handles[id(goal_handle)] = goal_handle

        for preempted_handle in preempted:
            preempted_handle._easyfleet_preempted = True
            self.on_preempted(preempted_handle)

        # Transitions ACCEPTED -> EXECUTING (skipped if already canceled)
        # and schedules `_dispatch_execute` as this goal's execute_callback
        # on the node's executor -- see the module docstring for why that
        # must be a MultiThreadedExecutor.
        goal_handle.execute()

    def _on_parameters_set(self, params) -> None:
        for param in params:
            if param.name == self._param_name:
                self._allow_preemption = bool(param.value)

    def _wait_for_turn(self, goal_handle) -> bool:
        """Wait until no other goal runs; False if this one must not run at all."""
        with self._idle:
            while True:
                stopped = (
                    self._shutting_down or self._halt_requested or not self._goals_enabled or
                    getattr(goal_handle, '_easyfleet_preempted', False))
                if stopped or not self._active_handles:
                    self._waiting_handles.pop(id(goal_handle), None)
                    if not stopped:
                        self._active_handles[id(goal_handle)] = goal_handle
                    self._idle.notify_all()
                    return not stopped
                self._idle.wait(0.05)

    def _dispatch_execute(self, goal_handle):
        if not self._wait_for_turn(goal_handle):
            # Superseded by a newer goal, or goals were disabled, before it started.
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                self.logger.warning(
                    f"Action '{self._action_name}': a queued goal was superseded before it "
                    'could start executing.')
                goal_handle.abort()
            return self._action_type.Result()

        self._thread_local.goal_handle = goal_handle
        try:
            if goal_handle.is_cancel_requested:
                # Canceled by the client before it ever got to run.
                goal_handle.canceled()
                return self._action_type.Result()

            self.logger.info(
                f"Capability '{self._action_name}' on robot '{self._robot_name}': "
                'goal execution started.')
            try:
                result = self.on_execute(goal_handle)
            except Exception as e:  # Never let a capability bug crash the executor.
                self.logger.error(
                    f"Action '{self._action_name}': exception raised from on_execute(): {e}")
                if goal_handle.is_active:
                    goal_handle.abort()
                result = self._action_type.Result()
            if goal_handle.is_active:
                self.logger.warning(
                    f"Action '{self._action_name}': on_execute() returned without "
                    'settling the goal; aborting it.')
                goal_handle.abort()
                result = self._action_type.Result()
            self.logger.info(
                f"Capability '{self._action_name}' on robot '{self._robot_name}': "
                'goal execution finished.')
            return result if result is not None else self._action_type.Result()
        finally:
            self._thread_local.goal_handle = None
            with self._idle:
                self._active_handles.pop(id(goal_handle), None)
                self._idle.notify_all()
