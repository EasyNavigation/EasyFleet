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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_easynav_deployment/easynav_navigation_capability.hpp"

using namespace std::chrono_literals;
using Navigation = easyfleet_interfaces::action::Navigation;
using easyfleet_easynav_deployment::EasynavNavigationActionServer;

namespace
{

std::string unique_test_name(const std::string & base)
{
  static std::atomic<uint64_t> counter{0};
  return base + "_" + std::to_string(counter.fetch_add(1));
}

std::thread spin_in_background(rclcpp::Executor & executor)
{
  std::thread thread([&executor] {executor.spin();});
  while (!executor.is_spinning()) {
    std::this_thread::yield();
  }
  return thread;
}

Navigation::Goal make_goal(const std::string & parameters_json)
{
  Navigation::Goal goal;
  goal.parameters_json = parameters_json;
  return goal;
}

}  // namespace

// Only exercises on_goal_received()'s parameters_json/goal_id validation
// (accept/reject at the action-server handshake), which never touches
// EasyNav -- no mock GoalManager needed for this suite.
class EasynavNavigationActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    action_name_ = unique_test_name("navigation_test");

    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter(action_name_ + ".waypoint_ids", std::vector<std::string>{"dock"}),
      rclcpp::Parameter(action_name_ + ".waypoints.dock.frame_id", "map"),
      rclcpp::Parameter(action_name_ + ".waypoints.dock.x", 1.0),
      rclcpp::Parameter(action_name_ + ".waypoints.dock.y", 2.0),
      rclcpp::Parameter(action_name_ + ".waypoints.dock.yaw", 0.0),
    });
    server_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      unique_test_name("test_easynav_server"), options);
    action_server_ = std::make_shared<EasynavNavigationActionServer>(
      *server_node_, action_name_);

    client_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_easynav_client"));
    client_ = rclcpp_action::create_client<Navigation>(client_node_, action_name_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node_->get_node_base_interface());
    executor_->add_node(client_node_);
    spin_thread_ = spin_in_background(*executor_);

    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  static void TearDownTestSuite()
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(server_node_->get_node_base_interface());
    executor_->remove_node(client_node_);
    executor_.reset();
    client_.reset();
    client_node_.reset();
    action_server_.reset();
    server_node_.reset();
  }

  /// Sends `goal`, waits for the accept/reject handshake, and returns
  /// whether it was accepted (a non-null goal handle).
  static bool was_accepted(const Navigation::Goal & goal)
  {
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(2s) != std::future_status::ready) {
      return false;
    }
    return static_cast<bool>(future.get());
  }

  inline static std::string action_name_;
  inline static rclcpp_lifecycle::LifecycleNode::SharedPtr server_node_;
  inline static std::shared_ptr<EasynavNavigationActionServer> action_server_;
  inline static rclcpp::Node::SharedPtr client_node_;
  inline static rclcpp_action::Client<Navigation>::SharedPtr client_;
  inline static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  inline static std::thread spin_thread_;
};

TEST_F(EasynavNavigationActionServerTest, RejectsEmptyParametersJson)
{
  EXPECT_FALSE(was_accepted(make_goal("")));
}

TEST_F(EasynavNavigationActionServerTest, RejectsMalformedJson)
{
  EXPECT_FALSE(was_accepted(make_goal("{not valid json")));
}

TEST_F(EasynavNavigationActionServerTest, RejectsJsonWithoutGoalId)
{
  EXPECT_FALSE(was_accepted(make_goal(R"({"other_field": "value"})")));
}

TEST_F(EasynavNavigationActionServerTest, RejectsUnknownWaypointId)
{
  EXPECT_FALSE(was_accepted(make_goal(R"({"goal_id": "no_such_waypoint"})")));
}

TEST_F(EasynavNavigationActionServerTest, AcceptsKnownWaypointIdThenCancels)
{
  auto goal_handle_future = client_->async_send_goal(make_goal(R"({"goal_id": "dock"})"));
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  // Cancel right away: this suite is only about the accept/reject
  // handshake, not about driving a full navigation (see test_bt_nodes.cpp
  // for that, with a mock GoalManager). Cancelling keeps this test fast
  // and avoids leaving a goal running against a non-existent EasyNav.
  auto cancel_future = client_->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
