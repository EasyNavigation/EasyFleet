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


#include <gtest/gtest.h>

#include <string>

#include "easyfleet_core/order_history.hpp"

namespace
{

using easyfleet_core::OrderHistory;
using OrderStatus = easyfleet_interfaces::msg::OrderStatus;

rclcpp_action::GoalUUID uuid(uint8_t seed)
{
  rclcpp_action::GoalUUID id{};
  for (std::size_t i = 0; i < id.size(); ++i) {
    id[i] = static_cast<uint8_t>(seed + i);
  }
  return id;
}

builtin_interfaces::msg::Time at(int32_t sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  return t;
}

action_msgs::msg::GoalStatusArray statuses(
  std::initializer_list<std::pair<rclcpp_action::GoalUUID, int8_t>> list)
{
  action_msgs::msg::GoalStatusArray msg;
  for (const auto & [id, state] : list) {
    action_msgs::msg::GoalStatus status;
    status.goal_info.goal_id.uuid = id;
    status.status = state;
    msg.status_list.push_back(status);
  }
  return msg;
}

TEST(OrderHistory, HexIs32LowercaseDigits)
{
  rclcpp_action::GoalUUID id{};
  id[0] = 0xab;
  id[15] = 0x0f;
  EXPECT_EQ(OrderHistory::to_hex(id), "ab00000000000000000000000000000f");
}

TEST(OrderHistory, AcceptedOrdersUseTheirOrderIdOrTheGoalUuid)
{
  OrderHistory history;
  EXPECT_FALSE(history.latest());
  EXPECT_EQ(history.accepted(uuid(1), "pick-7", "/r/manipulation", at(3)), "pick-7");
  const auto hex = OrderHistory::to_hex(uuid(2));
  EXPECT_EQ(history.accepted(uuid(2), "", "/r/manipulation", at(4)), hex);

  const auto pick = history.find("pick-7");
  ASSERT_TRUE(pick);
  EXPECT_EQ(pick->goal_id, OrderHistory::to_hex(uuid(1)));
  EXPECT_EQ(pick->action_name, "/r/manipulation");
  EXPECT_EQ(pick->state, OrderStatus::STATE_ACCEPTED);
  EXPECT_EQ(pick->accepted_stamp.sec, 3);
  EXPECT_EQ(pick->finished_stamp.sec, 0);
  ASSERT_TRUE(history.find(hex));
  EXPECT_EQ(history.latest()->order_id, hex);
  EXPECT_FALSE(history.find("nope"));
  EXPECT_FALSE(history.find(""));
}

TEST(OrderHistory, FollowsTheStatesAndStampsTheFinish)
{
  OrderHistory history;
  history.accepted(uuid(1), "a", "/r/nav", at(1));
  history.update(statuses({{uuid(1), OrderStatus::STATE_EXECUTING}}), at(2));
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_EXECUTING);
  EXPECT_EQ(history.find("a")->finished_stamp.sec, 0);
  history.update(statuses({{uuid(1), OrderStatus::STATE_CANCELING}}), at(3));
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_CANCELING);
  history.update(statuses({{uuid(1), OrderStatus::STATE_CANCELED}}), at(4));
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_CANCELED);
  EXPECT_EQ(history.find("a")->finished_stamp.sec, 4);
}

TEST(OrderHistory, AFinishedOrderNeverChangesAgain)
{
  OrderHistory history;
  history.accepted(uuid(1), "a", "/r/nav", at(1));
  history.update(statuses({{uuid(1), OrderStatus::STATE_SUCCEEDED}}), at(5));
  // The same status array is published again with later changes of others.
  history.update(statuses({{uuid(1), OrderStatus::STATE_ABORTED}}), at(9));
  history.update(statuses({{uuid(1), OrderStatus::STATE_SUCCEEDED}}), at(9));
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_SUCCEEDED);
  EXPECT_EQ(history.find("a")->finished_stamp.sec, 5);
}

TEST(OrderHistory, IgnoresUnknownGoalsAndUnknownStates)
{
  OrderHistory history;
  history.accepted(uuid(1), "a", "/r/nav", at(1));
  history.update(
    statuses({{uuid(9), OrderStatus::STATE_SUCCEEDED}, {uuid(1), OrderStatus::STATE_UNKNOWN}}),
    at(2));
  EXPECT_EQ(history.size(), 1u);
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_ACCEPTED);
  history.update(action_msgs::msg::GoalStatusArray(), at(3));
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_ACCEPTED);
}

TEST(OrderHistory, SeveralGoalsInOneStatusMessage)
{
  OrderHistory history;
  history.accepted(uuid(1), "a", "/r/nav", at(1));
  history.accepted(uuid(2), "b", "/r/nav", at(2));
  history.update(
    statuses({{uuid(1), OrderStatus::STATE_ABORTED}, {uuid(2), OrderStatus::STATE_EXECUTING}}),
    at(3));
  EXPECT_EQ(history.find("a")->state, OrderStatus::STATE_ABORTED);
  EXPECT_EQ(history.find("b")->state, OrderStatus::STATE_EXECUTING);
}

TEST(OrderHistory, ARepeatedOrderIdFindsTheNewestOrder)
{
  OrderHistory history;
  history.accepted(uuid(1), "retry", "/r/nav", at(1));
  history.update(statuses({{uuid(1), OrderStatus::STATE_ABORTED}}), at(2));
  history.accepted(uuid(2), "retry", "/r/nav", at(3));
  const auto order = history.find("retry");
  ASSERT_TRUE(order);
  EXPECT_EQ(order->goal_id, OrderHistory::to_hex(uuid(2)));
  EXPECT_EQ(order->state, OrderStatus::STATE_ACCEPTED);
  // The old goal's states still go to the old order.
  history.update(statuses({{uuid(1), OrderStatus::STATE_SUCCEEDED}}), at(4));
  EXPECT_EQ(history.find("retry")->state, OrderStatus::STATE_ACCEPTED);
}

TEST(OrderHistory, KeepsTheNewestAndDropsFinishedOrdersFirst)
{
  OrderHistory history(3);
  history.accepted(uuid(1), "1", "/r/nav", at(1));  // stays running
  history.accepted(uuid(2), "2", "/r/nav", at(2));
  history.accepted(uuid(3), "3", "/r/nav", at(3));
  history.update(
    statuses({{uuid(2), OrderStatus::STATE_SUCCEEDED}, {uuid(3), OrderStatus::STATE_ABORTED}}),
    at(4));
  history.accepted(uuid(4), "4", "/r/nav", at(5));
  EXPECT_EQ(history.size(), 3u);
  EXPECT_TRUE(history.find("1")) << "running orders outlive finished ones";
  EXPECT_FALSE(history.find("2")) << "the oldest finished one goes";
  EXPECT_TRUE(history.find("3"));
  EXPECT_TRUE(history.find("4"));
}

TEST(OrderHistory, WithoutFinishedOrdersTheOldestGoes)
{
  OrderHistory history(2);
  history.accepted(uuid(1), "1", "/r/nav", at(1));
  history.accepted(uuid(2), "2", "/r/nav", at(2));
  history.accepted(uuid(3), "3", "/r/nav", at(3));
  EXPECT_EQ(history.size(), 2u);
  EXPECT_FALSE(history.find("1"));
  EXPECT_EQ(history.latest()->order_id, "3");
}

TEST(OrderHistory, CapacityIsAtLeastOne)
{
  OrderHistory history(0);
  history.accepted(uuid(1), "1", "/r/nav", at(1));
  history.accepted(uuid(2), "2", "/r/nav", at(2));
  EXPECT_EQ(history.size(), 1u);
  EXPECT_EQ(history.latest()->order_id, "2");
}

TEST(OrderHistory, TerminalStates)
{
  EXPECT_FALSE(OrderHistory::is_terminal(OrderStatus::STATE_UNKNOWN));
  EXPECT_FALSE(OrderHistory::is_terminal(OrderStatus::STATE_ACCEPTED));
  EXPECT_FALSE(OrderHistory::is_terminal(OrderStatus::STATE_EXECUTING));
  EXPECT_FALSE(OrderHistory::is_terminal(OrderStatus::STATE_CANCELING));
  EXPECT_TRUE(OrderHistory::is_terminal(OrderStatus::STATE_SUCCEEDED));
  EXPECT_TRUE(OrderHistory::is_terminal(OrderStatus::STATE_CANCELED));
  EXPECT_TRUE(OrderHistory::is_terminal(OrderStatus::STATE_ABORTED));
}

TEST(OrderHistory, StatesMatchActionMsgsGoalStatus)
{
  EXPECT_EQ(OrderStatus::STATE_EXECUTING, action_msgs::msg::GoalStatus::STATUS_EXECUTING);
  EXPECT_EQ(OrderStatus::STATE_CANCELING, action_msgs::msg::GoalStatus::STATUS_CANCELING);
  EXPECT_EQ(OrderStatus::STATE_SUCCEEDED, action_msgs::msg::GoalStatus::STATUS_SUCCEEDED);
  EXPECT_EQ(OrderStatus::STATE_CANCELED, action_msgs::msg::GoalStatus::STATUS_CANCELED);
  EXPECT_EQ(OrderStatus::STATE_ABORTED, action_msgs::msg::GoalStatus::STATUS_ABORTED);
}

}  // namespace
