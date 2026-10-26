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

"""OrderHistory, mirroring easyfleet_core's order_history.hpp/.cpp."""

import copy
import threading

from easyfleet_interfaces.msg import OrderStatus

_TERMINAL = (OrderStatus.STATE_SUCCEEDED, OrderStatus.STATE_CANCELED, OrderStatus.STATE_ABORTED)


def is_terminal(state: int) -> bool:
    """Return whether a state is terminal (succeeded, canceled or aborted)."""
    return state in _TERMINAL


def to_hex(goal_id) -> str:
    """Return a goal UUID (16 bytes) as 32 lowercase hex digits."""
    return ''.join(f'{int(b):02x}' for b in goal_id)


class OrderHistory:
    """
    The recent orders (goals) of one capability and their states.

    Orders enter when their goal is accepted and follow the states the
    action server publishes on its status topic. Only the newest
    `capacity` are kept, dropping finished ones first. Thread-safe.
    """

    def __init__(self, capacity: int = 20):
        self._capacity = max(int(capacity), 1)
        self._lock = threading.Lock()
        self._orders: list[OrderStatus] = []  # oldest first

    def accepted(self, goal_id, order_id: str, action_name: str, stamp) -> str:
        """Record an accepted goal; return the order id it was recorded with."""
        order = OrderStatus()
        order.goal_id = to_hex(goal_id)
        order.order_id = order_id or order.goal_id
        order.action_name = action_name
        order.state = OrderStatus.STATE_ACCEPTED
        order.accepted_stamp = stamp
        with self._lock:
            self._orders.append(order)
            self._trim()
        return order.order_id

    def update(self, msg, stamp) -> None:
        """Take the states of a GoalStatusArray; finished orders never change again."""
        with self._lock:
            for status in msg.status_list:
                goal_id = to_hex(status.goal_info.goal_id.uuid)
                order = next((o for o in reversed(self._orders) if o.goal_id == goal_id), None)
                if (order is None or is_terminal(order.state) or
                        status.status == OrderStatus.STATE_UNKNOWN):
                    continue
                order.state = status.status
                if is_terminal(order.state):
                    order.finished_stamp = stamp
            self._trim()

    def find(self, order_id: str):
        """Return a copy of the newest order with this order id, or None."""
        with self._lock:
            order = next((o for o in reversed(self._orders) if o.order_id == order_id), None)
            return copy.deepcopy(order) if order is not None else None

    def latest(self):
        """Return a copy of the newest order, or None."""
        with self._lock:
            return copy.deepcopy(self._orders[-1]) if self._orders else None

    def __len__(self) -> int:
        """Return how many orders are kept now."""
        with self._lock:
            return len(self._orders)

    def _trim(self) -> None:
        while len(self._orders) > self._capacity:
            # The oldest finished order goes first; the oldest one if none finished.
            index = next(
                (i for i, o in enumerate(self._orders) if is_terminal(o.state)), 0)
            del self._orders[index]
