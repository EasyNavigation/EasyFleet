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
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "easynav_interfaces/msg/navigation_control.hpp"
#include "easynav_system/GoalManagerClient.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easyfleet_easynav_navigation/bt_nodes/navigate.hpp"

using namespace std::chrono_literals;

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

/// Ticks `tree` at 20 Hz until it leaves RUNNING or `timeout` elapses.
BT::NodeStatus tick_until_done(BT::Tree & tree, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(50ms);
  }
  return status;
}

}  // namespace

class NavigateTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_navigate_client"));
    server_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_navigate_server"));

    executor_.add_node(client_node_);
    executor_.add_node(server_node_);
    spin_thread_ = spin_in_background(executor_);

    gm_client_ = easynav::GoalManagerClient::make_shared(client_node_);

    auto waypoints = std::make_shared<std::map<std::string, geometry_msgs::msg::PoseStamped>>();
    geometry_msgs::msg::PoseStamped dock;
    dock.header.frame_id = "map";
    dock.pose.position.x = 1.0;
    (*waypoints)["dock"] = dock;
    waypoints_ = waypoints;

    factory_.registerNodeType<easyfleet_easynav_navigation::Navigate>("Navigate");
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(client_node_);
    executor_.remove_node(server_node_);
  }

  /// Creates a blackboard with "waypoints"/"gm_client" already set, since
  /// Navigate (a BT.CPP plugin) reads both from there rather than from
  /// constructor arguments -- see bt_nodes/navigate.hpp.
  BT::Blackboard::Ptr make_blackboard()
  {
    auto blackboard = BT::Blackboard::create();
    blackboard->set("waypoints", waypoints_);
    blackboard->set("gm_client", gm_client_);
    return blackboard;
  }

  /// Mocks the EasyNav GoalManager: accepts any REQUEST addressed to
  /// `gm_client_` and, after a couple of FEEDBACK messages, reports FINISHED.
  void start_mock_goal_manager()
  {
    server_sub_ = server_node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
      "easynav_control", rclcpp::QoS(10).reliable(),
      [this](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
        if (msg->type != easynav_interfaces::msg::NavigationControl::REQUEST) {
          return;
        }
        const std::string requester_id = msg->user_id;

        easynav_interfaces::msg::NavigationControl accept;
        accept.type = easynav_interfaces::msg::NavigationControl::ACCEPT;
        accept.user_id = "mock_goal_manager";
        accept.nav_current_user_id = requester_id;
        server_pub_->publish(accept);

        easynav_interfaces::msg::NavigationControl finished;
        finished.type = easynav_interfaces::msg::NavigationControl::FINISHED;
        finished.user_id = "mock_goal_manager";
        finished.nav_current_user_id = requester_id;
        server_pub_->publish(finished);
      });
    server_pub_ = server_node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
      "easynav_control", rclcpp::QoS(10).reliable());
  }

  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  easynav::GoalManagerClient::SharedPtr gm_client_;
  std::shared_ptr<const std::map<std::string, geometry_msgs::msg::PoseStamped>> waypoints_;
  BT::BehaviorTreeFactory factory_;

  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr server_sub_;
  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr server_pub_;
};

TEST_F(NavigateTest, FailsImmediatelyWithoutGoalIdPort)
{
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><Navigate/></BehaviorTree></root>)",
    make_blackboard());
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(NavigateTest, FailsImmediatelyForUnknownWaypoint)
{
  auto blackboard = make_blackboard();
  blackboard->set("goal_id", std::string("no_such_waypoint"));
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
    R"(<Navigate goal_id="{goal_id}"/></BehaviorTree></root>)",
    blackboard);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}

TEST_F(NavigateTest, SucceedsWhenEasyNavReportsFinished)
{
  start_mock_goal_manager();

  auto blackboard = make_blackboard();
  blackboard->set("goal_id", std::string("dock"));
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
    R"(<Navigate goal_id="{goal_id}"/></BehaviorTree></root>)",
    blackboard);

  const auto status = tick_until_done(tree, 5s);
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(gm_client_->get_state(), easynav::GoalManagerClient::State::NAVIGATION_FINISHED);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
