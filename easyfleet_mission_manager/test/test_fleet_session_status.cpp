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


#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "easyfleet_core/robot_status_publisher.hpp"
#include "easyfleet_mission_manager/fleet_session.hpp"
#include "easyfleet_mission_manager/mission_helpers.hpp"
#include "easyfleet_mission_manager/robot_handle.hpp"

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "test_nav_fake_capability.hpp"

using namespace std::chrono_literals;
using easyfleet_interfaces::msg::CapabilityStatus;
using easyfleet_interfaces::msg::FleetHeartbeat;
using easyfleet_interfaces::msg::OrderStatus;
using easyfleet_interfaces::msg::RobotStatus;
using easyfleet_mission_manager::make_navigation_goal;
using easyfleet_mission_manager::Navigation;
using easyfleet_mission_manager_test::TestNavCapability;

namespace
{

std::string unique_name(const std::string & prefix)
{
  static std::atomic<uint64_t> counter{0};
  return prefix + "_" + std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1));
}

std::string make_capabilities_file()
{
  static std::atomic<uint64_t> counter{0};
  const std::string path = "/tmp/easyfleet_mission_manager_test_status_" +
    std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1)) + ".json";
  std::ofstream(path) << R"({"name":"navigation"})";
  return path;
}

bool wait_until(const std::function<bool()> & condition, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (condition()) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return condition();
}

/// One navigation capability under /<robot>, on its own executor.
class FakeRobot
{
public:
  FakeRobot(
    const std::string & robot, double goal_s, std::vector<rclcpp::Parameter> extra = {})
  {
    rclcpp::NodeOptions options;
    options.arguments({"--ros-args", "-r", "__ns:=/" + robot});
    extra.emplace_back("capabilities_file", make_capabilities_file());
    extra.emplace_back("navigation.mock_duration_seconds", goal_s);
    options.parameter_overrides(extra);
    capability_ = std::make_shared<TestNavCapability>("navigation", options);
    executor_.add_node(capability_->get_node_base_interface());
    spin_thread_ = std::thread([this] {executor_.spin();});
    capability_->configure_node();
    capability_->activate_node();
  }

  ~FakeRobot()
  {
    executor_.cancel();
    spin_thread_.join();
  }

  std::shared_ptr<TestNavCapability> capability() const {return capability_;}

private:
  std::shared_ptr<TestNavCapability> capability_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

Navigation::Goal goal_with_order(const std::string & order_id)
{
  auto goal = make_navigation_goal("dock");
  goal.order_id = order_id;
  return goal;
}

/// Publishes hand-made /robot_status messages, as another robot process would.
class StatusSource
{
public:
  StatusSource()
  : node_(std::make_shared<rclcpp::Node>(unique_name("status_source")))
  {
    pub_ = node_->create_publisher<RobotStatus>("/robot_status", rclcpp::QoS(10).reliable());
  }

  void publish(const RobotStatus & msg) {pub_->publish(msg);}

  size_t subscribers() const {return pub_->get_subscription_count();}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<RobotStatus>::SharedPtr pub_;
};

RobotStatus status_of(
  const std::string & robot, const std::vector<std::string> & capabilities, int32_t sec)
{
  RobotStatus msg;
  msg.robot = robot;
  msg.stamp.sec = sec;
  for (const auto & name : capabilities) {
    CapabilityStatus capability;
    capability.robot = robot;
    capability.capability = name;
    capability.action_name = "/" + robot + "/" + name;
    msg.capabilities.push_back(capability);
  }
  return msg;
}

}  // namespace

TEST(FleetHeartbeatTest, TheSessionPublishesItOnceASecond)
{
  easyfleet::FleetSession session;
  auto node = std::make_shared<rclcpp::Node>(unique_name("heartbeat_listener"));
  std::mutex mutex;
  std::vector<FleetHeartbeat> heard;
  auto sub = node->create_subscription<FleetHeartbeat>(
    "/fleet_heartbeat", rclcpp::QoS(10).best_effort(),
    [&](const FleetHeartbeat & msg) {
      std::lock_guard<std::mutex> lock(mutex);
      heard.push_back(msg);
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 3500ms) {
    executor.spin_some(100ms);
  }
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_GE(heard.size(), 2u);
  EXPECT_LE(heard.size(), 5u);
  ASSERT_FALSE(heard.empty());
  EXPECT_EQ(heard.front().source, "/mission_control");
}

TEST(FleetHeartbeatTest, CapabilitiesHearTheSessionAndLoseItWhenItGoes)
{
  const auto robot_name = unique_name("robot");
  FakeRobot robot(
    robot_name, 30.0,
    {rclcpp::Parameter("navigation.link_timeout", 2.5),
      rclcpp::Parameter("navigation.on_link_lost", "cancel")});
  auto server = robot.capability()->get_action_server();
  {
    easyfleet::FleetSession session;
    easyfleet::RobotHandle handle(robot_name);
    session.add_robot(handle);
    session.discover_capabilities();
    ASSERT_TRUE(
      wait_until(
        [&] {
          const auto caps = handle.capabilities();
          return caps.size() == 1 && caps[0].link_state == CapabilityStatus::LINK_OK;
        }, 5s)) << "the heartbeat reports the link as OK";

    handle.run_capability<Navigation>("navigation", goal_with_order("long"), 60s);
    ASSERT_TRUE(wait_until([&] {return server->is_active();}, 5s));
  }
  // The session is gone: no more heartbeat, so the goal is stopped.
  ASSERT_TRUE(wait_until([&] {return !server->is_active();}, 6s));
  EXPECT_EQ(robot.capability()->get_status().link_state, CapabilityStatus::LINK_LOST);
  const auto order = robot.capability()->get_order_history().find("long");
  ASSERT_TRUE(order);
  EXPECT_TRUE(
    wait_until(
      [&] {
        return robot.capability()->get_order_history().find("long")->state ==
               OrderStatus::STATE_ABORTED;
      }, 3s));
}

TEST(GetOrderTest, TheSessionFindsAnOrderOfARobotWithItsState)
{
  const auto robot_name = unique_name("robot");
  FakeRobot robot(robot_name, 0.05);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle(robot_name);
  session.add_robot(handle);
  session.discover_capabilities();
  ASSERT_TRUE(handle.has_capability("navigation"));

  handle.run_capability<Navigation>("navigation", goal_with_order("deliver-9"), 5s);
  ASSERT_TRUE(wait_until([&] {return !handle.is_capability_running("navigation");}, 5s));
  ASSERT_TRUE(
    wait_until(
      [&] {
        const auto order = handle.get_order("deliver-9");
        return order && order->state == OrderStatus::STATE_SUCCEEDED;
      }, 5s));
  const auto order = session.get_order(robot_name, "deliver-9");
  ASSERT_TRUE(order);
  EXPECT_EQ(order->action_name, "/" + robot_name + "/navigation");
  EXPECT_EQ(order->goal_id.size(), 32u);

  EXPECT_FALSE(handle.get_order("never-sent"));
  // The heartbeat carries the last order too.
  ASSERT_TRUE(
    wait_until(
      [&] {
        const auto caps = handle.capabilities();
        return caps.size() == 1 && caps[0].order_id == "deliver-9" &&
               caps[0].order_state == OrderStatus::STATE_SUCCEEDED;
      }, 3s));
}

TEST(GetOrderTest, UnknownRobotsAndDetachedHandlesHaveNoOrders)
{
  easyfleet::FleetSession session;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(session.get_order("nobody", "x"));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 500ms) << "no capability to ask";
  easyfleet::RobotHandle detached("detached");
  EXPECT_FALSE(detached.get_order("x"));
  EXPECT_FALSE(detached.status());
}

TEST(GetOrderTest, ACapabilityThatStoppedAnsweringTimesOut)
{
  const auto robot_name = unique_name("robot");
  auto robot = std::make_unique<FakeRobot>(robot_name, 0.0);
  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle(robot_name);
  session.add_robot(handle);
  session.discover_capabilities();
  ASSERT_TRUE(handle.has_capability("navigation"));
  robot.reset();  // the registry still lists it for a few seconds
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(handle.get_order("x", 500ms));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}

TEST(RobotStatusMergeTest, NothingBeforeAnyStatusOfTheRobot)
{
  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle(unique_name("robot"));
  session.add_robot(handle);
  EXPECT_FALSE(handle.status());
}

TEST(RobotStatusMergeTest, TheProcessesOfARobotAreMerged)
{
  const auto robot = unique_name("robot");
  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle(robot);
  session.add_robot(handle);
  StatusSource base, arm;
  ASSERT_TRUE(wait_until([&] {return base.subscribers() > 0 && arm.subscribers() > 0;}, 5s));

  auto base_msg = status_of(robot, {"navigation"}, 10);
  base_msg.has_pose = true;
  base_msg.pose.pose.pose.position.x = 3.0;
  auto arm_msg = status_of(robot, {"manipulation", "perception"}, 12);
  arm_msg.has_battery = true;
  arm_msg.battery.percentage = 0.5f;
  base.publish(base_msg);
  arm.publish(arm_msg);
  // Another robot is not mixed in.
  base.publish(status_of(unique_name("other"), {"navigation"}, 99));

  ASSERT_TRUE(
    wait_until([&] {return handle.status() && handle.status()->capabilities.size() == 3;}, 5s));
  auto status = *handle.status();
  EXPECT_EQ(status.robot, robot);
  EXPECT_EQ(status.stamp.sec, 12) << "the newest stamp";
  EXPECT_TRUE(status.has_pose);
  EXPECT_DOUBLE_EQ(status.pose.pose.pose.position.x, 3.0);
  EXPECT_TRUE(status.has_battery);
  EXPECT_FLOAT_EQ(status.battery.percentage, 0.5f);

  // A later message without pose keeps the last known one; an older stamp
  // does not move the robot's stamp back.
  base.publish(status_of(robot, {"navigation"}, 11));
  std::this_thread::sleep_for(300ms);
  status = *handle.status();
  EXPECT_TRUE(status.has_pose);
  EXPECT_EQ(status.stamp.sec, 12);
}

TEST(RobotStatusMergeTest, CapabilitiesNotReportedForAWhileAreDropped)
{
  const auto robot = unique_name("robot");
  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle(robot);
  session.add_robot(handle);
  StatusSource source;
  ASSERT_TRUE(wait_until([&] {return source.subscribers() > 0;}, 5s));
  source.publish(status_of(robot, {"navigation", "perception"}, 1));
  ASSERT_TRUE(
    wait_until([&] {return handle.status() && handle.status()->capabilities.size() == 2;}, 5s));
  // Only navigation keeps being reported.
  const auto start = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start < 4s) {
    source.publish(status_of(robot, {"navigation"}, 2));
    std::this_thread::sleep_for(250ms);
  }
  const auto status = *handle.status();
  ASSERT_EQ(status.capabilities.size(), 1u);
  EXPECT_EQ(status.capabilities[0].capability, "navigation");
}

TEST(RobotStatusMergeTest, ADeploymentStyleProcessIsSeenEndToEnd)
{
  const auto robot = unique_name("robot");
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("robot_status.rate_hz", 5.0)});
  auto node = std::make_shared<rclcpp::Node>("deployment_config", "/" + robot, options);
  FakeRobot fake(robot, 0.0);
  easyfleet_core::RobotStatusPublisher publisher(
    node, [&]() {return std::vector<CapabilityStatus>{fake.capability()->get_status()};});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin([&] {executor.spin();});

  easyfleet::FleetSession session;
  easyfleet::RobotHandle handle(robot);
  session.add_robot(handle);
  const bool seen = wait_until(
    [&] {return handle.status() && handle.status()->capabilities.size() == 1;}, 5s);
  executor.cancel();
  spin.join();
  ASSERT_TRUE(seen);
  EXPECT_EQ(handle.status()->capabilities[0].action_name, "/" + robot + "/navigation");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
