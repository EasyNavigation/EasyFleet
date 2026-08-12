// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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
#include <cstdio>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arch_mockup/capability.hpp"
#include "arch_mockup_interfaces/msg/capability_description.hpp"
#include "arch_mockup_interfaces/msg/capability_status.hpp"
#include "test_capability_server.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using arch_mockup_interfaces::msg::CapabilityDescription;
using arch_mockup_interfaces::msg::CapabilityStatus;
using arch_mockup_test::Fibonacci;
using arch_mockup_test::spin_in_background;
using arch_mockup_test::TestCapabilityActionServer;
using arch_mockup_test::unique_test_name;
using arch_mockup_test::wait_until;
using TestCapability = arch_mockup::Capability<TestCapabilityActionServer>;

namespace
{

/// Thread-safe collector of received messages, used to make assertions
/// about published content.
template<typename MsgT>
class Collector
{
public:
  void callback(const typename MsgT::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.push_back(*msg);
  }

  std::vector<MsgT> messages() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }

  std::size_t count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_.size();
  }

private:
  mutable std::mutex mutex_;
  std::vector<MsgT> messages_;
};

std::string make_temp_file(const std::string & content)
{
  static std::atomic<uint64_t> counter{0};
  const std::string path = "/tmp/arch_mockup_test_capabilities_" +
    std::to_string(::getpid()) + "_" + std::to_string(counter.fetch_add(1)) + ".json";
  std::ofstream file(path);
  file << content;
  return path;
}

}  // namespace

class CapabilityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    capability_name_ = unique_test_name("test_capability");
    capability_ = std::make_shared<TestCapability>(capability_name_);
    sub_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_capability_subscriber"));

    executor_.add_node(capability_->get_node_base_interface());
    executor_.add_node(sub_node_);
    spin_thread_ = spin_in_background(executor_);
  }

  void TearDown() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    description_subs_.clear();
    status_subs_.clear();
    executor_.remove_node(capability_->get_node_base_interface());
    executor_.remove_node(sub_node_);
    capability_.reset();
    sub_node_.reset();
    for (const auto & path : temp_files_) {
      std::remove(path.c_str());
    }
  }

  std::string make_capabilities_file(const std::string & content)
  {
    const std::string path = make_temp_file(content);
    temp_files_.push_back(path);
    return path;
  }

  std::shared_ptr<Collector<CapabilityDescription>> subscribe_descriptions()
  {
    auto collector = std::make_shared<Collector<CapabilityDescription>>();
    auto sub = sub_node_->create_subscription<CapabilityDescription>(
      "/capabilities", rclcpp::QoS(1).reliable().transient_local(),
      [collector](const CapabilityDescription::SharedPtr msg) {collector->callback(msg);});
    description_subs_.push_back(sub);
    return collector;
  }

  std::shared_ptr<Collector<CapabilityStatus>> subscribe_status()
  {
    auto collector = std::make_shared<Collector<CapabilityStatus>>();
    auto sub = sub_node_->create_subscription<CapabilityStatus>(
      "/capabilities_status", rclcpp::QoS(10).reliable(),
      [collector](const CapabilityStatus::SharedPtr msg) {collector->callback(msg);});
    status_subs_.push_back(sub);
    return collector;
  }

  std::string capability_name_;
  std::shared_ptr<TestCapability> capability_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> description_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> status_subs_;
  std::vector<std::string> temp_files_;
};

TEST_F(CapabilityTest, StartsUnconfigured)
{
  EXPECT_EQ(
    capability_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(CapabilityTest, ConfigureSucceedsAndMovesToInactive)
{
  auto cb = TestCapability::CallbackReturn::FAILURE;
  const auto & state = capability_->configure(cb);
  EXPECT_EQ(cb, TestCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(CapabilityTest, ActivateWithoutFileParameterFails)
{
  capability_->configure();
  auto cb = TestCapability::CallbackReturn::SUCCESS;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, TestCapability::CallbackReturn::FAILURE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(CapabilityTest, ActivateWithMissingFileFails)
{
  capability_->set_parameter(
    rclcpp::Parameter("capabilities_file", "/no/such/file/exists.json"));
  capability_->configure();
  auto cb = TestCapability::CallbackReturn::SUCCESS;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, TestCapability::CallbackReturn::FAILURE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(CapabilityTest, ActivatePublishesFileContentsAndIdentityOnCapabilitiesTopic)
{
  const std::string json = R"({"name":"test","skills":["a","b"]})";
  const auto path = make_capabilities_file(json);
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_descriptions();

  auto cb = TestCapability::CallbackReturn::FAILURE;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, TestCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
  const CapabilityDescription msg = collector->messages().front();
  EXPECT_EQ(msg.description_json, json);
  EXPECT_EQ(msg.capability, capability_name_);
  EXPECT_EQ(msg.robot, "");
  EXPECT_EQ(msg.action_name, "/" + capability_name_);
}

TEST_F(CapabilityTest, LateSubscriberAfterActivateStillGetsCapabilitiesMessage)
{
  const std::string json = R"({"name":"late-join"})";
  const auto path = make_capabilities_file(json);
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();
  capability_->activate();

  // Subscribe only *after* the message has already been published: transient
  // local durability should still deliver it.
  auto collector = subscribe_descriptions();

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
  EXPECT_EQ(collector->messages().front().description_json, json);
}

TEST_F(CapabilityTest, HeartbeatPublishesIdentityEverySecondWhileActive)
{
  const auto path = make_capabilities_file("{}");
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_status();

  capability_->activate();

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 2;}, 3s));
  for (const auto & msg : collector->messages()) {
    EXPECT_EQ(msg.capability, capability_name_);
    EXPECT_EQ(msg.robot, "");
    EXPECT_EQ(msg.action_name, "/" + capability_name_);
    EXPECT_FALSE(msg.busy) << "no goal was ever sent, so this capability should be idle";
  }
}

TEST_F(CapabilityTest, DeactivateStopsTheHeartbeat)
{
  const auto path = make_capabilities_file("{}");
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_status();

  capability_->activate();
  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));

  capability_->deactivate();
  const auto count_after_deactivate = collector->count();

  std::this_thread::sleep_for(1500ms);
  EXPECT_EQ(collector->count(), count_after_deactivate);
}

TEST_F(CapabilityTest, CleanupThenReconfigureAndActivateAgainWorks)
{
  const auto path = make_capabilities_file(R"({"round":1})");
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();
  capability_->activate();
  capability_->deactivate();
  capability_->cleanup();
  EXPECT_EQ(
    capability_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto collector = subscribe_descriptions();

  auto cb = TestCapability::CallbackReturn::FAILURE;
  capability_->configure();
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, TestCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
  EXPECT_EQ(collector->messages().back().description_json, R"({"round":1})");
}

TEST_F(CapabilityTest, GetActionServerExposesAWorkingActionServer)
{
  auto action_server = capability_->get_action_server();
  ASSERT_TRUE(action_server);
  EXPECT_EQ(action_server->get_action_name(), capability_name_);

  auto client_node = std::make_shared<rclcpp::Node>(unique_test_name("test_capability_client"));
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, capability_name_);
  executor_.add_node(client_node);
  ASSERT_TRUE(client->wait_for_action_server(5s));

  Fibonacci::Goal goal;
  goal.order = 3;
  auto goal_handle_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  executor_.remove_node(client_node);
}

TEST_F(CapabilityTest, CapabilityNameIsUsedAsNodeName)
{
  EXPECT_EQ(std::string(capability_->get_name()), capability_name_);
  EXPECT_EQ(capability_->get_capability_name(), capability_name_);
}

TEST_F(CapabilityTest, DefaultsToEmptyRobotNameWithoutANamespace)
{
  EXPECT_EQ(capability_->get_robot_name(), "");
  EXPECT_EQ(capability_->get_resolved_action_name(), "/" + capability_name_);
}

TEST(CapabilityStandaloneTest, HeartbeatReportsBusyWhileAGoalIsExecuting)
{
  const auto name = unique_test_name("test_capability_busy");
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    // Long enough to span at least one 1s heartbeat tick while still
    // executing, short enough to keep the test fast.
    rclcpp::Parameter(name + ".mock_delay_seconds", 1.5),
  });
  auto capability = std::make_shared<TestCapability>(name, options);

  const auto path = make_temp_file("{}");
  capability->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability->configure();

  auto sub_node = std::make_shared<rclcpp::Node>(unique_test_name("test_capability_busy_sub"));
  auto client_node =
    std::make_shared<rclcpp::Node>(unique_test_name("test_capability_busy_client"));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(capability->get_node_base_interface());
  executor.add_node(sub_node);
  executor.add_node(client_node);
  auto spin_thread = spin_in_background(executor);

  capability->activate();

  auto collector = std::make_shared<Collector<CapabilityStatus>>();
  auto status_sub = sub_node->create_subscription<CapabilityStatus>(
    "/capabilities_status", rclcpp::QoS(10).reliable(),
    [collector](const CapabilityStatus::SharedPtr msg) {collector->callback(msg);});

  auto client = rclcpp_action::create_client<Fibonacci>(client_node, name);
  ASSERT_TRUE(client->wait_for_action_server(5s));
  Fibonacci::Goal goal;
  goal.order = 3;
  auto goal_handle_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  // The next heartbeat (~1s after activation) should land while the 1.5s
  // goal is still executing.
  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 3s));
  EXPECT_TRUE(collector->messages().back().busy) << "a goal is executing right now";

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  // The heartbeat after the goal settles should report idle again.
  ASSERT_TRUE(wait_until([&] {return collector->count() >= 2;}, 3s));
  EXPECT_FALSE(collector->messages().back().busy) << "the goal has already finished";

  executor.cancel();
  spin_thread.join();
  executor.remove_node(capability->get_node_base_interface());
  executor.remove_node(sub_node);
  executor.remove_node(client_node);
  std::remove(path.c_str());
}

TEST(CapabilityStandaloneTest, RobotAndActionNameReflectTheNamespaceAtRuntime)
{
  const auto name = unique_test_name("test_capability_ns");
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/robot1"});

  auto capability = std::make_shared<TestCapability>(name, options);

  EXPECT_EQ(capability->get_robot_name(), "robot1");
  EXPECT_EQ(capability->get_resolved_action_name(), "/robot1/" + name);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
