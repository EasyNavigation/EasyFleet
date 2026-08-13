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

#include "easyfleet_fake_alone_deployment/manipulation_fake_capability.hpp"
#include "easyfleet_interfaces/msg/capability_description.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_interfaces::msg::CapabilityDescription;
using easyfleet_interfaces::msg::CapabilityStatus;
using Manipulation = easyfleet_interfaces::action::Manipulation;
using easyfleet_fake_alone_deployment::ManipulationFakeCapability;
using easyfleet_fake_alone_deployment_test::spin_in_background;
using easyfleet_fake_alone_deployment_test::unique_test_name;
using easyfleet_fake_alone_deployment_test::wait_until;

namespace
{

// Capability packages no longer ship a default capabilities JSON of their
// own (that lives under the deployment scenarios in
// src/EasyFleet/easyfleet_fake_alone_deployment), so tests exercise the
// publish-file-verbatim behavior against a JSON they write themselves.
constexpr char kSampleCapabilitiesJson[] =
  R"json({
  "name": "manipulation",
  "display_name": "Reach a joint target",
  "action": {
    "name": "/manipulation",
    "type": "easyfleet_interfaces/action/Manipulation"
  },
  "requirements": [
    "The target joint configuration must already be free of collisions."
  ],
  "effects": [
    "On success, the manipulator's joints end up at the requested target."
  ],
  "parameters": {
    "manipulation.allow_preemption": {
      "type": "bool",
      "default": true
    }
  }
}
)json";

std::string write_sample_capabilities_file()
{
  const auto path = "/tmp/" + unique_test_name("manipulation_capabilities") + ".json";
  std::ofstream file(path);
  file << kSampleCapabilitiesJson;
  return path;
}

Manipulation::Goal make_joint_target_goal()
{
  Manipulation::Goal goal;
  goal.mode = Manipulation::Goal::MODE_JOINT_TARGET;
  goal.joint_target.name = {"joint1"};
  goal.joint_target.position = {1.0};
  return goal;
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

// ManipulationFakeCapability's node/action name ("manipulation") is fixed by
// design (it isn't parameterized), so unlike other fixtures in this project
// the capability is created and fully torn down per test rather than shared
// across the suite.
class ManipulationFakeCapabilityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // ManipulationFakeActionServer reads its mock timing parameters once, at
    // construction time, so they must be overridden via NodeOptions (not
    // set_parameter() afterwards) to actually take effect. Keeping the
    // whole suite on a short mock duration also keeps it fast.
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter("manipulation.mock_execution_duration", 0.5),
      rclcpp::Parameter("manipulation.mock_feedback_period", 0.1),
    });
    capability_ = std::make_shared<ManipulationFakeCapability>(options);
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

  std::shared_ptr<ManipulationFakeCapability> capability_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> description_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> status_subs_;
};

TEST_F(ManipulationFakeCapabilityTest, StartsUnconfigured)
{
  EXPECT_EQ(
    capability_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ManipulationFakeCapabilityTest, NodeAndActionAreNamedManipulation)
{
  EXPECT_EQ(std::string(capability_->get_name()), "manipulation");
  EXPECT_EQ(capability_->get_capability_name(), "manipulation");
  ASSERT_TRUE(capability_->get_action_server());
  EXPECT_EQ(capability_->get_action_server()->get_action_name(), "manipulation");
  EXPECT_EQ(capability_->get_resolved_action_name(), "/manipulation");
  EXPECT_EQ(capability_->get_robot_name(), "");
}

TEST_F(ManipulationFakeCapabilityTest, ActivateWithoutCapabilitiesFileParameterFails)
{
  capability_->configure();
  auto cb = ManipulationFakeCapability::CallbackReturn::SUCCESS;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, ManipulationFakeCapability::CallbackReturn::FAILURE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ManipulationFakeCapabilityTest, ActivatePublishesTheProvidedCapabilitiesJsonVerbatim)
{
  const auto path = write_sample_capabilities_file();

  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_descriptions();

  auto cb = ManipulationFakeCapability::CallbackReturn::FAILURE;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, ManipulationFakeCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
  const CapabilityDescription msg = collector->messages().front();
  EXPECT_EQ(msg.description_json, kSampleCapabilitiesJson);
  EXPECT_EQ(msg.capability, "manipulation");
  EXPECT_EQ(msg.robot, "");
  EXPECT_EQ(msg.action_name, "/manipulation");
}

TEST_F(ManipulationFakeCapabilityTest, PublishedCapabilitiesJsonIsWellFormed)
{
  // Not a full JSON parser: checks the shape an LLM/planner consuming this
  // file would expect, without adding a JSON parsing dependency.
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_descriptions();
  capability_->activate();
  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));

  const std::string json = collector->messages().front().description_json;
  EXPECT_NE(json.find("\"name\""), std::string::npos);
  EXPECT_NE(json.find("\"action\""), std::string::npos);
  EXPECT_NE(json.find("Manipulation"), std::string::npos);
  EXPECT_NE(json.find("\"requirements\""), std::string::npos);
  EXPECT_NE(json.find("\"effects\""), std::string::npos);
  EXPECT_NE(json.find("\"parameters\""), std::string::npos);
  const auto last_non_space = json.find_last_not_of(" \t\r\n");
  EXPECT_EQ(json.front(), '{');
  ASSERT_NE(last_non_space, std::string::npos);
  EXPECT_EQ(json[last_non_space], '}');
}

TEST_F(ManipulationFakeCapabilityTest, HeartbeatPublishesIdentityEverySecondWhileActive)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_status();
  capability_->activate();

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 2;}, 3s));
  for (const auto & msg : collector->messages()) {
    EXPECT_EQ(msg.capability, "manipulation");
    EXPECT_EQ(msg.robot, "");
    EXPECT_EQ(msg.action_name, "/manipulation");
    EXPECT_FALSE(msg.busy);
  }
}

TEST_F(ManipulationFakeCapabilityTest, DeactivateStopsTheHeartbeat)
{
  const auto path = write_sample_capabilities_file();
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

TEST_F(ManipulationFakeCapabilityTest, CleanupThenReconfigureAndActivateAgainWorks)
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

  auto cb = ManipulationFakeCapability::CallbackReturn::FAILURE;
  capability_->configure();
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, ManipulationFakeCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
}

TEST_F(ManipulationFakeCapabilityTest, ActiveCapabilityCompletesARealJointTargetGoal)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();
  capability_->activate();

  auto client = rclcpp_action::create_client<Manipulation>(sub_node_, "manipulation");
  ASSERT_TRUE(client->wait_for_action_server(5s));

  auto goal = make_joint_target_goal();

  auto goal_handle_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wrapped.result);
  EXPECT_EQ(wrapped.result->error_code, Manipulation::Result::SUCCESS);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
