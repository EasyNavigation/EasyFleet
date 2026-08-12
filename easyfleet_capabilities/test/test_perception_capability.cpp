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

#include "easyfleet_capabilities/perception_capability.hpp"
#include "easyfleet_interfaces/msg/capability_description.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_interfaces::msg::CapabilityDescription;
using easyfleet_interfaces::msg::CapabilityStatus;
using Perception = easyfleet_interfaces::action::Perception;
using easyfleet_capabilities::PerceptionCapability;
using easyfleet_capabilities_test::spin_in_background;
using easyfleet_capabilities_test::unique_test_name;
using easyfleet_capabilities_test::wait_until;

namespace
{

// Capability packages no longer ship a default capabilities JSON of their
// own (that lives under the deployment scenarios in
// src/EasyFleet/easyfleet_example_deployments), so tests exercise the
// publish-file-verbatim behavior against a JSON they write themselves.
constexpr char kSampleCapabilitiesJson[] =
  R"json({
  "name": "perception",
  "display_name": "Detect objects of a known class",
  "action": {
    "name": "/perception",
    "type": "easyfleet_interfaces/action/Perception"
  },
  "requirements": [
    "The requested 'object_classes' list must include the target class this mock supports."
  ],
  "effects": [
    "None on the physical world: this is a perception-only capability."
  ],
  "parameters": {
    "perception.allow_preemption": {
      "type": "bool",
      "default": true
    }
  }
}
)json";

std::string write_sample_capabilities_file()
{
  const auto path = "/tmp/" + unique_test_name("perception_capabilities") + ".json";
  std::ofstream file(path);
  file << kSampleCapabilitiesJson;
  return path;
}

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

}  // namespace

// PerceptionCapability's node/action name ("perception") is fixed by design
// (it isn't parameterized), so the capability is created and fully torn
// down per test rather than shared across the suite.
class PerceptionCapabilityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter("perception.detection_rate_hz", 50.0),
    });
    capability_ = std::make_shared<PerceptionCapability>(options);
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

  std::shared_ptr<PerceptionCapability> capability_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> description_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> status_subs_;
};

TEST_F(PerceptionCapabilityTest, StartsUnconfigured)
{
  EXPECT_EQ(
    capability_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PerceptionCapabilityTest, NodeAndActionAreNamedPerception)
{
  EXPECT_EQ(std::string(capability_->get_name()), "perception");
  EXPECT_EQ(capability_->get_capability_name(), "perception");
  ASSERT_TRUE(capability_->get_action_server());
  EXPECT_EQ(capability_->get_action_server()->get_action_name(), "perception");
  EXPECT_EQ(capability_->get_resolved_action_name(), "/perception");
  EXPECT_EQ(capability_->get_robot_name(), "");
}

TEST_F(PerceptionCapabilityTest, ActivateWithoutCapabilitiesFileParameterFails)
{
  capability_->configure();
  auto cb = PerceptionCapability::CallbackReturn::SUCCESS;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, PerceptionCapability::CallbackReturn::FAILURE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(PerceptionCapabilityTest, ActivatePublishesTheProvidedCapabilitiesJsonVerbatim)
{
  const auto path = write_sample_capabilities_file();

  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_descriptions();

  auto cb = PerceptionCapability::CallbackReturn::FAILURE;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, PerceptionCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
  const CapabilityDescription msg = collector->messages().front();
  EXPECT_EQ(msg.description_json, kSampleCapabilitiesJson);
  EXPECT_EQ(msg.capability, "perception");
  EXPECT_EQ(msg.robot, "");
  EXPECT_EQ(msg.action_name, "/perception");
}

TEST_F(PerceptionCapabilityTest, HeartbeatPublishesIdentityEverySecondWhileActive)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_status();
  capability_->activate();

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 2;}, 3s));
  for (const auto & msg : collector->messages()) {
    EXPECT_EQ(msg.capability, "perception");
    EXPECT_EQ(msg.robot, "");
    EXPECT_EQ(msg.action_name, "/perception");
    EXPECT_FALSE(msg.busy);
  }
}

TEST_F(PerceptionCapabilityTest, CleanupThenReconfigureAndActivateAgainWorks)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();
  capability_->activate();
  capability_->deactivate();
  capability_->cleanup();
  EXPECT_EQ(
    capability_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  auto collector = subscribe_descriptions();

  auto cb = PerceptionCapability::CallbackReturn::FAILURE;
  capability_->configure();
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, PerceptionCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
}

TEST_F(PerceptionCapabilityTest, ActiveCapabilityStreamsDetectionsUntilCanceled)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();
  capability_->activate();

  auto client = rclcpp_action::create_client<Perception>(sub_node_, "perception");
  ASSERT_TRUE(client->wait_for_action_server(5s));

  Perception::Goal goal;
  goal.object_classes.push_back("gato");

  std::atomic<int> feedback_count{0};
  rclcpp_action::Client<Perception>::SendGoalOptions options;
  options.feedback_callback =
    [&feedback_count](
    rclcpp_action::ClientGoalHandle<Perception>::SharedPtr,
    const std::shared_ptr<const Perception::Feedback>)
    {
      feedback_count.fetch_add(1);
    };

  auto goal_handle_future = client->async_send_goal(goal, options);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  ASSERT_TRUE(wait_until([&] {return feedback_count.load() >= 5;}, 2s));

  auto cancel_future = client->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(wrapped.result);
  EXPECT_EQ(wrapped.result->error_code, Perception::Result::CANCELED);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
