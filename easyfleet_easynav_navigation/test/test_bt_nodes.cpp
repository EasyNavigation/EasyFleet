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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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

/// Polls `pred` until it returns true or `timeout` elapses.
template<typename Predicate>
bool wait_until(Predicate pred, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return pred();
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
    geometry_msgs::msg::PoseStamped home;
    home.header.frame_id = "map";
    home.pose.position.x = 5.0;
    (*waypoints)["home"] = home;
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

  /// Mocks a GoalManager that only records REQUESTs; the test answers.
  void start_recording_goal_manager()
  {
    server_pub_ = server_node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
      "easynav_control", rclcpp::QoS(10).reliable());
    server_sub_ = server_node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
      "easynav_control", rclcpp::QoS(10).reliable(),
      [this](easynav_interfaces::msg::NavigationControl::UniquePtr msg) {
        if (msg->type != easynav_interfaces::msg::NavigationControl::REQUEST) {
          return;
        }
        std::lock_guard<std::mutex> lock(requests_mutex_);
        requests_.push_back(*msg);
      });
  }

  std::vector<easynav_interfaces::msg::NavigationControl> requests()
  {
    std::lock_guard<std::mutex> lock(requests_mutex_);
    return requests_;
  }

  void answer(uint8_t type, const std::string & requester)
  {
    easynav_interfaces::msg::NavigationControl msg;
    msg.type = type;
    msg.user_id = "mock_goal_manager";
    msg.nav_current_user_id = requester;
    server_pub_->publish(msg);
  }

  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr server_sub_;
  std::mutex requests_mutex_;
  std::vector<easynav_interfaces::msg::NavigationControl> requests_;
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

TEST_F(NavigateTest, FailsWhenEasyNavNeverAnswers)
{
  auto blackboard = make_blackboard();
  blackboard->set("goal_id", std::string("dock"));
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
    R"(<Navigate goal_id="{goal_id}" response_timeout="0.5"/></BehaviorTree></root>)",
    blackboard);

  const auto start = std::chrono::steady_clock::now();
  const auto status = tick_until_done(tree, 5s);
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 3s);
}

TEST_F(NavigateTest, GoalChangeWhileWaitingForAcceptanceIsSentOnceAccepted)
{
  start_recording_goal_manager();
  auto blackboard = make_blackboard();
  blackboard->set("goal_id", std::string("dock"));
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
    R"(<Navigate goal_id="{goal_id}" response_timeout="10"/></BehaviorTree></root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(wait_until([this] {return requests().size() == 1u;}, 2s));
  const auto requester = requests()[0].user_id;

  // Not accepted yet: the change must wait, not be dropped.
  blackboard->set("goal_id", std::string("home"));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(requests().size(), 1u);

  answer(easynav_interfaces::msg::NavigationControl::ACCEPT, requester);
  ASSERT_TRUE(
    wait_until(
      [this] {
        return gm_client_->get_state() ==
               easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING;
      }, 2s));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(wait_until([this] {return requests().size() == 2u;}, 2s));
  ASSERT_FALSE(requests()[1].goals.goals.empty());
  EXPECT_DOUBLE_EQ(requests()[1].goals.goals[0].pose.position.x, 5.0);

  answer(easynav_interfaces::msg::NavigationControl::ACCEPT, requester);
  answer(easynav_interfaces::msg::NavigationControl::FINISHED, requester);
  EXPECT_EQ(tick_until_done(tree, 5s), BT::NodeStatus::SUCCESS);
}

TEST_F(NavigateTest, StartWhileTheClientAwaitsAnAnswerSendsOnceAnswered)
{
  start_recording_goal_manager();
  const std::string xml =
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
    R"(<Navigate goal_id="{goal_id}" response_timeout="10"/></BehaviorTree></root>)";
  auto first_bb = make_blackboard();
  first_bb->set("goal_id", std::string("dock"));
  auto first = factory_.createTreeFromText(xml, first_bb);
  EXPECT_EQ(first.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(wait_until([this] {return requests().size() == 1u;}, 2s));
  const auto requester = requests()[0].user_id;

  // A new goal (new tree, same client) before EasyNav answered the first.
  auto second_bb = make_blackboard();
  second_bb->set("goal_id", std::string("home"));
  auto second = factory_.createTreeFromText(xml, second_bb);
  EXPECT_EQ(second.tickOnce(), BT::NodeStatus::RUNNING);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(requests().size(), 1u) << "must wait, not be ignored by the client";

  answer(easynav_interfaces::msg::NavigationControl::ACCEPT, requester);
  ASSERT_TRUE(
    wait_until(
      [this] {
        return gm_client_->get_state() ==
               easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING;
      }, 2s));
  EXPECT_EQ(second.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(wait_until([this] {return requests().size() == 2u;}, 2s));
  EXPECT_DOUBLE_EQ(requests()[1].goals.goals[0].pose.position.x, 5.0);
}

TEST_F(NavigateTest, FailsWhenEasyNavReportsFailed)
{
  start_recording_goal_manager();
  auto blackboard = make_blackboard();
  blackboard->set("goal_id", std::string("dock"));
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="4"><BehaviorTree ID="Test">)"
    R"(<Navigate goal_id="{goal_id}"/></BehaviorTree></root>)",
    blackboard);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(wait_until([this] {return requests().size() == 1u;}, 2s));
  const auto requester = requests()[0].user_id;
  answer(easynav_interfaces::msg::NavigationControl::ACCEPT, requester);
  answer(easynav_interfaces::msg::NavigationControl::FAILED, requester);
  EXPECT_EQ(tick_until_done(tree, 5s), BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
