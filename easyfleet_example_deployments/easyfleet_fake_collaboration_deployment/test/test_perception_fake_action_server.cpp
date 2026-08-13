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
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easyfleet_fake_collaboration_deployment/perception_fake_capability.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using Perception = easyfleet_interfaces::action::Perception;
using easyfleet_fake_collaboration_deployment::PerceptionFakeActionServer;
using easyfleet_fake_collaboration_deployment_test::spin_in_background;
using easyfleet_fake_collaboration_deployment_test::unique_test_name;
using easyfleet_fake_collaboration_deployment_test::wait_until;

namespace
{

Perception::Goal make_goal(const std::string & object_class = "gato")
{
  Perception::Goal goal;
  goal.object_classes.push_back(object_class);
  return goal;
}

}  // namespace

// Mock detection rate is fixed fast (via parameter overrides at node
// construction) for the whole suite, so tests collecting several feedback
// messages stay quick.
class PerceptionFakeActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    action_name_ = unique_test_name("perception_test");

    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter(action_name_ + ".detection_rate_hz", 50.0),
      rclcpp::Parameter(action_name_ + ".target_class", "gato"),
    });
    server_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      unique_test_name("test_perception_server"), options);
    action_server_ = std::make_shared<PerceptionFakeActionServer>(server_node_.get(), action_name_);

    client_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_perception_client"));
    client_ = rclcpp_action::create_client<Perception>(client_node_, action_name_);

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

  inline static std::string action_name_;
  inline static rclcpp_lifecycle::LifecycleNode::SharedPtr server_node_;
  inline static std::shared_ptr<PerceptionFakeActionServer> action_server_;
  inline static rclcpp::Node::SharedPtr client_node_;
  inline static rclcpp_action::Client<Perception>::SharedPtr client_;
  inline static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  inline static std::thread spin_thread_;
};

TEST_F(PerceptionFakeActionServerTest, ExposesConfiguredActionName)
{
  EXPECT_EQ(action_server_->get_action_name(), action_name_);
}

TEST_F(PerceptionFakeActionServerTest, RejectsGoalWithEmptyObjectClassesList)
{
  Perception::Goal goal;
  auto future = client_->async_send_goal(goal);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get());
}

TEST_F(PerceptionFakeActionServerTest, RejectsGoalForUnsupportedClass)
{
  auto future = client_->async_send_goal(make_goal("perro"));
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get());
}

TEST_F(PerceptionFakeActionServerTest, AcceptsTargetClassCaseInsensitiveAndAsSubstring)
{
  auto handle_future = client_->async_send_goal(make_goal("Gatos"));
  ASSERT_EQ(handle_future.wait_for(2s), std::future_status::ready);
  auto handle = handle_future.get();
  ASSERT_TRUE(handle);

  ASSERT_TRUE(wait_until([this] {return action_server_->is_active();}, 1s));
  auto cancel_future = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result_future = client_->async_get_result(handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(PerceptionFakeActionServerTest, StreamsOneToThreeDetectionsInBoth2dAnd3dUntilCanceled)
{
  std::mutex mutex;
  std::vector<std::shared_ptr<const Perception::Feedback>> feedbacks;
  rclcpp_action::Client<Perception>::SendGoalOptions options;
  options.feedback_callback =
    [&mutex, &feedbacks](
    rclcpp_action::ClientGoalHandle<Perception>::SharedPtr,
    const std::shared_ptr<const Perception::Feedback> feedback)
    {
      std::lock_guard<std::mutex> lock(mutex);
      feedbacks.push_back(feedback);
    };

  auto handle_future = client_->async_send_goal(make_goal(), options);
  ASSERT_EQ(handle_future.wait_for(2s), std::future_status::ready);
  auto handle = handle_future.get();
  ASSERT_TRUE(handle);

  ASSERT_TRUE(
    wait_until(
      [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return feedbacks.size() >= 5;
      }, 2s));

  auto cancel_future = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto result_future = client_->async_get_result(handle);
  ASSERT_EQ(result_future.wait_for(2s), std::future_status::ready);
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(wrapped.result);
  EXPECT_EQ(wrapped.result->error_code, Perception::Result::CANCELED);

  std::lock_guard<std::mutex> lock(mutex);
  for (const auto & feedback : feedbacks) {
    ASSERT_GE(feedback->detections_3d.detections.size(), 1u);
    ASSERT_LE(feedback->detections_3d.detections.size(), 3u);
    EXPECT_EQ(feedback->detections_2d.detections.size(), feedback->detections_3d.detections.size());
    for (const auto & detection : feedback->detections_3d.detections) {
      ASSERT_FALSE(detection.results.empty());
      EXPECT_EQ(detection.results.front().hypothesis.class_id, "gato");
    }
    for (const auto & detection : feedback->detections_2d.detections) {
      ASSERT_FALSE(detection.results.empty());
      EXPECT_EQ(detection.results.front().hypothesis.class_id, "gato");
    }
  }
}

TEST_F(PerceptionFakeActionServerTest, NewGoalPreemptsRunningGoalByDefault)
{
  EXPECT_TRUE(action_server_->is_preemptable());

  auto first_handle_future = client_->async_send_goal(make_goal());
  ASSERT_EQ(first_handle_future.wait_for(2s), std::future_status::ready);
  auto first_handle = first_handle_future.get();
  ASSERT_TRUE(first_handle);
  ASSERT_TRUE(wait_until([this] {return action_server_->is_active();}, 1s));

  auto first_result_future = client_->async_get_result(first_handle);

  auto second_handle_future = client_->async_send_goal(make_goal());
  ASSERT_EQ(second_handle_future.wait_for(2s), std::future_status::ready);
  auto second_handle = second_handle_future.get();
  ASSERT_TRUE(second_handle) << "second goal should be accepted since preemption is allowed";

  ASSERT_EQ(first_result_future.wait_for(2s), std::future_status::ready);
  auto first_wrapped = first_result_future.get();
  EXPECT_EQ(first_wrapped.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(first_wrapped.result);
  EXPECT_EQ(first_wrapped.result->error_code, Perception::Result::ABORTED);

  ASSERT_TRUE(wait_until([this] {return action_server_->is_active();}, 1s));
  auto cancel_future = client_->async_cancel_goal(second_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto second_result_future = client_->async_get_result(second_handle);
  ASSERT_EQ(second_result_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(second_result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST(PerceptionFakeActionServerStandaloneTest, NonPreemptableRejectsSecondGoalWhileBusy)
{
  const auto action_name = unique_test_name("perception_np");
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter(action_name + ".allow_preemption", false),
    rclcpp::Parameter(action_name + ".detection_rate_hz", 50.0),
  });
  auto server_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    unique_test_name("test_perception_server_np"), options);
  auto action_server = std::make_shared<PerceptionFakeActionServer>(server_node.get(), action_name);
  ASSERT_FALSE(action_server->is_preemptable());

  auto client_node = std::make_shared<rclcpp::Node>(unique_test_name("test_perception_client_np"));
  auto client = rclcpp_action::create_client<Perception>(client_node, action_name);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_node_base_interface());
  executor.add_node(client_node);
  auto spin_thread = spin_in_background(executor);

  ASSERT_TRUE(client->wait_for_action_server(5s));

  auto first_handle_future = client->async_send_goal(make_goal());
  ASSERT_EQ(first_handle_future.wait_for(2s), std::future_status::ready);
  auto first_handle = first_handle_future.get();
  ASSERT_TRUE(first_handle);
  ASSERT_TRUE(wait_until([&] {return action_server->is_active();}, 1s));

  auto second_handle_future = client->async_send_goal(make_goal());
  ASSERT_EQ(second_handle_future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(second_handle_future.get())
    << "second goal should have been rejected while the first is busy";

  auto cancel_future = client->async_cancel_goal(first_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);
  auto first_result_future = client->async_get_result(first_handle);
  ASSERT_EQ(first_result_future.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().code, rclcpp_action::ResultCode::CANCELED);

  executor.cancel();
  spin_thread.join();
  executor.remove_node(server_node->get_node_base_interface());
  executor.remove_node(client_node);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
