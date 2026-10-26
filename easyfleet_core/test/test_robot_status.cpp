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

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/robot_status_publisher.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_core::RobotStatusPublisher;
using easyfleet_core_test::spin_in_background;
using easyfleet_core_test::unique_test_name;
using easyfleet_core_test::wait_until;
using easyfleet_interfaces::msg::CapabilityStatus;
using easyfleet_interfaces::msg::RobotStatus;

namespace
{

std::vector<CapabilityStatus> two_capabilities()
{
  CapabilityStatus nav;
  nav.capability = "navigation";
  nav.busy = true;
  CapabilityStatus arm;
  arm.capability = "manipulation";
  return {nav, arm};
}

rclcpp::Node::SharedPtr make_node(
  const std::string & ns, std::vector<rclcpp::Parameter> params = {})
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(params);
  return std::make_shared<rclcpp::Node>(unique_test_name("status"), ns, options);
}

class RobotStatusTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    other_ = std::make_shared<rclcpp::Node>(unique_test_name("status_peer"));
    sub_ = other_->create_subscription<RobotStatus>(
      "/robot_status", rclcpp::QoS(10).reliable(),
      [this](const RobotStatus & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        received_.push_back(msg);
      });
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  void spin(const rclcpp::Node::SharedPtr & node)
  {
    executor_.add_node(node);
    executor_.add_node(other_);
    spin_thread_ = spin_in_background(executor_);
  }

  std::vector<RobotStatus> received(const std::string & robot)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<RobotStatus> out;
    for (const auto & msg : received_) {
      if (msg.robot == robot) {
        out.push_back(msg);
      }
    }
    return out;
  }

  rclcpp::Node::SharedPtr other_;
  rclcpp::Subscription<RobotStatus>::SharedPtr sub_;
  std::mutex mutex_;
  std::vector<RobotStatus> received_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

TEST_F(RobotStatusTest, ReportsTheRobotAndItsCapabilities)
{
  auto node = make_node("/robot_a");
  RobotStatusPublisher publisher(node, two_capabilities);
  const auto status = publisher.make_status();
  EXPECT_EQ(status.robot, "robot_a");
  ASSERT_EQ(status.capabilities.size(), 2u);
  EXPECT_EQ(status.capabilities[0].capability, "navigation");
  EXPECT_TRUE(status.capabilities[0].busy);
  EXPECT_FALSE(status.has_pose);
  EXPECT_FALSE(status.has_battery);
  EXPECT_GT(rclcpp::Time(status.stamp).nanoseconds(), 0);
}

TEST_F(RobotStatusTest, WithoutNamespaceOrCollectorItIsEmpty)
{
  auto node = make_node("/");
  RobotStatusPublisher publisher(node, nullptr);
  const auto status = publisher.make_status();
  EXPECT_EQ(status.robot, "");
  EXPECT_TRUE(status.capabilities.empty());
}

TEST_F(RobotStatusTest, PublishesAtTheConfiguredRate)
{
  auto node = make_node("/robot_rate", {rclcpp::Parameter("robot_status.rate_hz", 10.0)});
  RobotStatusPublisher publisher(node, two_capabilities);
  spin(node);
  ASSERT_TRUE(wait_until([&] {return received("robot_rate").size() >= 5;}, 3s));
  EXPECT_EQ(received("robot_rate").back().capabilities.size(), 2u);
}

TEST_F(RobotStatusTest, RateZeroDisablesPublishing)
{
  auto node = make_node("/robot_quiet", {rclcpp::Parameter("robot_status.rate_hz", 0.0)});
  RobotStatusPublisher publisher(node, two_capabilities);
  spin(node);
  std::this_thread::sleep_for(1500ms);
  EXPECT_TRUE(received("robot_quiet").empty());
  publisher.publish();
  ASSERT_TRUE(wait_until([&] {return received("robot_quiet").size() == 1;}, 3s));
}

TEST_F(RobotStatusTest, ReportsTheLastPoseAndBatteryOfItsTopics)
{
  auto node = make_node(
    "/robot_b", {rclcpp::Parameter("robot_status.pose_topic", "amcl_pose"),
      rclcpp::Parameter("robot_status.battery_topic", "battery")});
  RobotStatusPublisher publisher(node, two_capabilities);
  auto pose_pub = other_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/robot_b/amcl_pose", rclcpp::QoS(1).reliable());
  auto battery_pub = other_->create_publisher<sensor_msgs::msg::BatteryState>(
    "/robot_b/battery", rclcpp::SensorDataQoS());
  spin(node);

  ASSERT_TRUE(
    wait_until(
      [&] {
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";
        pose.pose.pose.position.x = 4.5;
        pose_pub->publish(pose);
        sensor_msgs::msg::BatteryState battery;
        battery.percentage = 0.75f;
        battery_pub->publish(battery);
        const auto s = publisher.make_status();
        return s.has_pose && s.has_battery;
      }, 5s, 100ms));
  const auto status = publisher.make_status();
  EXPECT_EQ(status.pose.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(status.pose.pose.pose.position.x, 4.5);
  EXPECT_FLOAT_EQ(status.battery.percentage, 0.75f);
}

TEST_F(RobotStatusTest, OnlyTheConfiguredTopicsAreReported)
{
  auto node = make_node("/robot_c", {rclcpp::Parameter("robot_status.pose_topic", "amcl_pose")});
  RobotStatusPublisher publisher(node, two_capabilities);
  auto battery_pub = other_->create_publisher<sensor_msgs::msg::BatteryState>(
    "/robot_c/battery", rclcpp::SensorDataQoS());
  spin(node);
  for (int i = 0; i < 5; ++i) {
    battery_pub->publish(sensor_msgs::msg::BatteryState());
    std::this_thread::sleep_for(100ms);
  }
  EXPECT_FALSE(publisher.make_status().has_battery);
  EXPECT_FALSE(publisher.make_status().has_pose) << "nothing published on it yet";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
