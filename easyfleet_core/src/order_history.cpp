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


#include "easyfleet_core/order_history.hpp"

#include <algorithm>
#include <cstdio>

namespace easyfleet_core
{

OrderHistory::OrderHistory(std::size_t capacity)
: capacity_(std::max<std::size_t>(capacity, 1))
{
}

std::string OrderHistory::to_hex(const rclcpp_action::GoalUUID & goal_id)
{
  std::string hex;
  hex.reserve(goal_id.size() * 2);
  char byte[3];
  for (const auto b : goal_id) {
    std::snprintf(byte, sizeof(byte), "%02x", b);
    hex += byte;
  }
  return hex;
}

bool OrderHistory::is_terminal(uint8_t state) noexcept
{
  return state == OrderStatus::STATE_SUCCEEDED || state == OrderStatus::STATE_CANCELED ||
         state == OrderStatus::STATE_ABORTED;
}

std::string OrderHistory::accepted(
  const rclcpp_action::GoalUUID & goal_id, const std::string & order_id,
  const std::string & action_name, const builtin_interfaces::msg::Time & stamp)
{
  OrderStatus order;
  order.goal_id = to_hex(goal_id);
  order.order_id = order_id.empty() ? order.goal_id : order_id;
  order.action_name = action_name;
  order.state = OrderStatus::STATE_ACCEPTED;
  order.accepted_stamp = stamp;

  std::lock_guard<std::mutex> lock(mutex_);
  orders_.push_back(order);
  trim();
  return order.order_id;
}

void OrderHistory::update(
  const action_msgs::msg::GoalStatusArray & msg, const builtin_interfaces::msg::Time & stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & status : msg.status_list) {
    const auto goal_id = to_hex(status.goal_info.goal_id.uuid);
    const auto it = std::find_if(
      orders_.rbegin(), orders_.rend(),
      [&goal_id](const OrderStatus & order) {return order.goal_id == goal_id;});
    if (it == orders_.rend() || is_terminal(it->state) ||
      status.status == OrderStatus::STATE_UNKNOWN)
    {
      continue;
    }
    it->state = status.status;
    if (is_terminal(it->state)) {
      it->finished_stamp = stamp;
    }
  }
  trim();
}

std::optional<OrderHistory::OrderStatus> OrderHistory::find(const std::string & order_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = std::find_if(
    orders_.rbegin(), orders_.rend(),
    [&order_id](const OrderStatus & order) {return order.order_id == order_id;});
  if (it == orders_.rend()) {
    return std::nullopt;
  }
  return *it;
}

std::optional<OrderHistory::OrderStatus> OrderHistory::latest() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (orders_.empty()) {
    return std::nullopt;
  }
  return orders_.back();
}

std::size_t OrderHistory::size() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return orders_.size();
}

void OrderHistory::trim()
{
  while (orders_.size() > capacity_) {
    // The oldest finished order goes first; the oldest one if none finished.
    auto it = std::find_if(
      orders_.begin(), orders_.end(),
      [](const OrderStatus & order) {return is_terminal(order.state);});
    orders_.erase(it == orders_.end() ? orders_.begin() : it);
  }
}

}  // namespace easyfleet_core
