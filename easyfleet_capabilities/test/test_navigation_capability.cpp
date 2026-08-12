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

#include "easyfleet_capabilities/navigation_capability.hpp"
#include "easyfleet_interfaces/msg/capability_description.hpp"
#include "easyfleet_interfaces/msg/capability_status.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_interfaces::msg::CapabilityDescription;
using easyfleet_interfaces::msg::CapabilityStatus;
using Navigation = easyfleet_interfaces::action::Navigation;
using easyfleet_capabilities::NavigationCapability;
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
  "name": "navigation",
  "display_name": "Navigate to a target pose",
  "action": {
    "name": "/navigation",
    "type": "easyfleet_interfaces/action/Navigation"
  },
  "requirements": [
    "The robot must already be localized within a known map."
  ],
  "effects": [
    "On success, the robot's physical location changes to the requested target pose."
  ],
  "parameters": {
    "navigation.allow_preemption": {
      "type": "bool",
      "default": true
    }
  }
}
)json";

std::string write_sample_capabilities_file()
{
  const auto path = "/tmp/" + unique_test_name("navigation_capabilities") + ".json";
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

// NavigationCapability's node/action name ("navigation") is fixed by design
// (it isn't parameterized), so unlike other fixtures in this project the
// capability is created and fully torn down per test rather than shared
// across the suite.
class NavigationCapabilityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // NavigationActionServer reads its mock timing parameters once, at
    // construction time, so they must be overridden via NodeOptions (not
    // set_parameter() afterwards) to actually take effect. Keeping the
    // whole suite on a short mock duration also keeps it fast.
    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter("navigation.mock_navigation_duration", 0.5),
      rclcpp::Parameter("navigation.mock_feedback_period", 0.1),
    });
    capability_ = std::make_shared<NavigationCapability>(options);
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

  std::shared_ptr<NavigationCapability> capability_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> description_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> status_subs_;
};

TEST_F(NavigationCapabilityTest, StartsUnconfigured)
{
  EXPECT_EQ(
    capability_->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(NavigationCapabilityTest, NodeAndActionAreNamedNavigation)
{
  EXPECT_EQ(std::string(capability_->get_name()), "navigation");
  EXPECT_EQ(capability_->get_capability_name(), "navigation");
  ASSERT_TRUE(capability_->get_action_server());
  EXPECT_EQ(capability_->get_action_server()->get_action_name(), "navigation");
  EXPECT_EQ(capability_->get_resolved_action_name(), "/navigation");
  EXPECT_EQ(capability_->get_robot_name(), "");
}

TEST_F(NavigationCapabilityTest, ActivateWithoutCapabilitiesFileParameterFails)
{
  capability_->configure();
  auto cb = NavigationCapability::CallbackReturn::SUCCESS;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, NavigationCapability::CallbackReturn::FAILURE);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(NavigationCapabilityTest, ActivatePublishesTheProvidedCapabilitiesJsonVerbatim)
{
  const auto path = write_sample_capabilities_file();

  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_descriptions();

  auto cb = NavigationCapability::CallbackReturn::FAILURE;
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, NavigationCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
  const CapabilityDescription msg = collector->messages().front();
  EXPECT_EQ(msg.description_json, kSampleCapabilitiesJson);
  EXPECT_EQ(msg.capability, "navigation");
  EXPECT_EQ(msg.robot, "");
  EXPECT_EQ(msg.action_name, "/navigation");
}

TEST_F(NavigationCapabilityTest, PublishedCapabilitiesJsonIsWellFormed)
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
  EXPECT_NE(json.find("Navigation"), std::string::npos);
  EXPECT_NE(json.find("\"requirements\""), std::string::npos);
  EXPECT_NE(json.find("\"effects\""), std::string::npos);
  EXPECT_NE(json.find("\"parameters\""), std::string::npos);
  const auto last_non_space = json.find_last_not_of(" \t\r\n");
  EXPECT_EQ(json.front(), '{');
  ASSERT_NE(last_non_space, std::string::npos);
  EXPECT_EQ(json[last_non_space], '}');
}

TEST_F(NavigationCapabilityTest, HeartbeatPublishesIdentityEverySecondWhileActive)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();

  auto collector = subscribe_status();
  capability_->activate();

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 2;}, 3s));
  for (const auto & msg : collector->messages()) {
    EXPECT_EQ(msg.capability, "navigation");
    EXPECT_EQ(msg.robot, "");
    EXPECT_EQ(msg.action_name, "/navigation");
    EXPECT_FALSE(msg.busy);
  }
}

TEST_F(NavigationCapabilityTest, DeactivateStopsTheHeartbeat)
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

TEST_F(NavigationCapabilityTest, CleanupThenReconfigureAndActivateAgainWorks)
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

  auto cb = NavigationCapability::CallbackReturn::FAILURE;
  capability_->configure();
  const auto & state = capability_->activate(cb);
  EXPECT_EQ(cb, NavigationCapability::CallbackReturn::SUCCESS);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_TRUE(wait_until([&] {return collector->count() >= 1;}, 2s));
}

TEST_F(NavigationCapabilityTest, ActiveCapabilityCompletesARealNavigationGoal)
{
  const auto path = write_sample_capabilities_file();
  capability_->set_parameter(rclcpp::Parameter("capabilities_file", path));
  capability_->configure();
  capability_->activate();

  auto client = rclcpp_action::create_client<Navigation>(sub_node_, "navigation");
  ASSERT_TRUE(client->wait_for_action_server(5s));

  Navigation::Goal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.orientation.w = 1.0;

  auto goal_handle_future = client->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wrapped.result);
  EXPECT_EQ(wrapped.result->error_code, Navigation::Result::SUCCESS);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
