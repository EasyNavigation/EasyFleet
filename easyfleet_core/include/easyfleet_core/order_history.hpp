// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the project EasyFleet
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef EASYFLEET_CORE__ORDER_HISTORY_HPP_
#define EASYFLEET_CORE__ORDER_HISTORY_HPP_

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "action_msgs/msg/goal_status_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp_action/types.hpp"

#include "easyfleet_interfaces/msg/order_status.hpp"

namespace easyfleet_core
{

/// @brief The recent orders (goals) of one capability and their states.
/**
 * Orders enter when their goal is accepted and follow the states the action
 * server publishes on its status topic. Only the newest `capacity` are kept,
 * dropping finished ones first. Thread-safe.
 */
class OrderHistory
{
public:
  using OrderStatus = easyfleet_interfaces::msg::OrderStatus;

  /// @param capacity How many orders to keep (at least 1).
  explicit OrderHistory(std::size_t capacity = 20);

  /// @brief Records an accepted goal.
  /// @param goal_id Its UUID.
  /// @param order_id Its order id; empty: the UUID in hex.
  /// @param action_name Fully-qualified action name.
  /// @param stamp When it was accepted.
  /// @return The order id it was recorded with.
  std::string accepted(
    const rclcpp_action::GoalUUID & goal_id, const std::string & order_id,
    const std::string & action_name, const builtin_interfaces::msg::Time & stamp);

  /// @brief Takes the states of a status message of the action server.
  /// Unknown goals are ignored and a finished order never changes again.
  /// @param msg The message.
  /// @param stamp Now, stamped on orders that finish with it.
  void update(
    const action_msgs::msg::GoalStatusArray & msg, const builtin_interfaces::msg::Time & stamp);

  /// @brief The newest order with this order id.
  [[nodiscard]] std::optional<OrderStatus> find(const std::string & order_id) const;

  /// @brief The newest order, if any.
  [[nodiscard]] std::optional<OrderStatus> latest() const;

  /// @brief How many orders are kept now.
  [[nodiscard]] std::size_t size() const;

  /// @brief Whether a state is terminal (succeeded, canceled or aborted).
  static bool is_terminal(uint8_t state) noexcept;

  /// @brief A goal UUID as 32 lowercase hex digits.
  static std::string to_hex(const rclcpp_action::GoalUUID & goal_id);

private:
  void trim();

  std::size_t capacity_;
  mutable std::mutex mutex_;
  /// Oldest first.
  std::deque<OrderStatus> orders_;
};

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__ORDER_HISTORY_HPP_
