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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_interfaces/msg/capability_description.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "easyfleet_mission_manager/capability_discovery.hpp"

using namespace std::chrono_literals;
using easyfleet_interfaces::msg::CapabilityDescription;
using easyfleet_interfaces::msg::CapabilityStatus;

namespace
{

std::string unique_name(const std::string & base)
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

}  // namespace

class CapabilityDiscoveryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    consumer_node_ = std::make_shared<rclcpp::Node>(unique_name("test_discovery_consumer"));
    publisher_node_ = std::make_shared<rclcpp::Node>(unique_name("test_discovery_publisher"));

    executor_.add_node(consumer_node_);
    executor_.add_node(publisher_node_);
    spin_thread_ = spin_in_background(executor_);

    capabilities_pub_ = publisher_node_->create_publisher<CapabilityDescription>(
      "/capabilities", rclcpp::QoS(10).reliable().transient_local());
    status_pub_ = publisher_node_->create_publisher<CapabilityStatus>(
      "/capabilities_status", rclcpp::QoS(10).reliable());
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(consumer_node_);
    executor_.remove_node(publisher_node_);
  }

  CapabilityDescription make_description(
    const std::string & capability, const std::string & action_name,
    const std::string & description_json, const std::string & robot = "")
  {
    CapabilityDescription msg;
    msg.robot = robot;
    msg.capability = capability;
    msg.action_name = action_name;
    msg.description_json = description_json;
    return msg;
  }

  // /capabilities_status is volatile (not transient-local), so a single
  // publish() issued before discover_capabilities() has created its
  // subscription is simply lost to normal discovery latency. Tests that
  // need a capability to be seen as "active" must keep republishing it
  // for as long as discovery is listening; this starts a repeating timer
  // that does so.
  rclcpp::TimerBase::SharedPtr start_heartbeat(
    const std::string & capability, const std::string & action_name,
    const std::string & robot = "", bool busy = false)
  {
    auto timer = publisher_node_->create_wall_timer(
      50ms,
      [this, capability, action_name, robot, busy] {
        CapabilityStatus msg;
        msg.robot = robot;
        msg.capability = capability;
        msg.action_name = action_name;
        msg.busy = busy;
        status_pub_->publish(msg);
      });
    timers_.push_back(timer);
    return timer;
  }

  rclcpp::Node::SharedPtr consumer_node_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<CapabilityDescription>::SharedPtr capabilities_pub_;
  rclcpp::Publisher<CapabilityStatus>::SharedPtr status_pub_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
};

TEST_F(CapabilityDiscoveryTest, ReturnsEmptyWhenNothingIsPublished)
{
  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 300ms);
  EXPECT_TRUE(result.empty());
}

TEST_F(CapabilityDiscoveryTest, ParsesJsonAndMarksActiveWhenHeartbeatSeen)
{
  capabilities_pub_->publish(
    make_description(
      "fake_cap", "/fake_cap", R"({"name":"fake_cap","display_name":"Fake Capability"})"));
  start_heartbeat("fake_cap", "/fake_cap");

  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 800ms);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().capability, "fake_cap");
  EXPECT_EQ(result.front().action_name, "/fake_cap");
  EXPECT_TRUE(result.front().description_json_valid);
  EXPECT_TRUE(result.front().active);
  EXPECT_FALSE(result.front().busy);
  EXPECT_EQ(result.front().description_json["display_name"], "Fake Capability");
}

TEST_F(CapabilityDiscoveryTest, TracksBusyStateFromTheLatestHeartbeat)
{
  capabilities_pub_->publish(make_description("fake_cap", "/fake_cap", R"({})"));
  start_heartbeat("fake_cap", "/fake_cap", "", /*busy=*/ true);

  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 800ms);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_TRUE(result.front().active);
  EXPECT_TRUE(result.front().busy);
}

TEST_F(CapabilityDiscoveryTest, CapabilityWithoutHeartbeatIsNotActive)
{
  capabilities_pub_->publish(make_description("silent_cap", "/silent_cap", R"({})"));

  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 500ms);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().capability, "silent_cap");
  EXPECT_TRUE(result.front().description_json_valid);
  EXPECT_FALSE(result.front().active);
}

TEST_F(CapabilityDiscoveryTest, MalformedJsonStillRegistersTheCapabilityButFlagsItInvalid)
{
  capabilities_pub_->publish(
    make_description("broken_cap", "/broken_cap", "not valid json {{{"));

  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 300ms);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result.front().capability, "broken_cap");
  EXPECT_FALSE(result.front().description_json_valid);
}

TEST_F(CapabilityDiscoveryTest, DiscoversMultipleDistinctCapabilities)
{
  capabilities_pub_->publish(make_description("cap_a", "/cap_a", R"({})"));
  capabilities_pub_->publish(make_description("cap_b", "/cap_b", R"({})"));
  start_heartbeat("cap_a", "/cap_a");

  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 800ms);

  ASSERT_EQ(result.size(), 2u);
  auto find_by_action_name = [&result](const std::string & action_name) {
      return std::find_if(
      result.begin(), result.end(),
        [&action_name](const easyfleet_mission_manager::CapabilityInfo & info) {
          return info.action_name == action_name;
        });
    };
  auto cap_a = find_by_action_name("/cap_a");
  auto cap_b = find_by_action_name("/cap_b");
  ASSERT_NE(cap_a, result.end());
  ASSERT_NE(cap_b, result.end());
  EXPECT_TRUE(cap_a->active);
  EXPECT_FALSE(cap_b->active);
}

TEST_F(CapabilityDiscoveryTest, SameCapabilityFromTwoRobotsAreKeptDistinctByActionName)
{
  capabilities_pub_->publish(
    make_description("navigation", "/robot1/navigation", R"({})", "robot1"));
  capabilities_pub_->publish(
    make_description("navigation", "/robot2/navigation", R"({})", "robot2"));
  start_heartbeat("navigation", "/robot1/navigation", "robot1");
  start_heartbeat("navigation", "/robot2/navigation", "robot2");

  auto result = easyfleet_mission_manager::discover_capabilities(*consumer_node_, 800ms);

  ASSERT_EQ(result.size(), 2u);
  for (const auto & info : result) {
    EXPECT_EQ(info.capability, "navigation");
    EXPECT_TRUE(info.active);
    EXPECT_TRUE(info.robot == "robot1" || info.robot == "robot2");
  }
  EXPECT_NE(result[0].action_name, result[1].action_name);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
