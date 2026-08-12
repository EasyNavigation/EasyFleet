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

#include "easyfleet_example_deployments/navigation_fake_capability.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using Navigation = easyfleet_interfaces::action::Navigation;
using easyfleet_example_deployments::NavigationFakeActionServer;
using easyfleet_example_deployments_test::spin_in_background;
using easyfleet_example_deployments_test::unique_test_name;
using easyfleet_example_deployments_test::wait_until;

namespace
{

geometry_msgs::msg::PoseStamped make_pose(const std::string & frame_id = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

// Mock timing is fixed short and fine-grained for the whole suite (via
// parameter overrides at node construction) so tests run quickly while
// still leaving enough steps to observe intermediate feedback reliably.
class NavigationFakeActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    action_name_ = unique_test_name("navigation_test");

    rclcpp::NodeOptions options;
    options.parameter_overrides(
    {
      rclcpp::Parameter(action_name_ + ".mock_navigation_duration", 1.0),
      rclcpp::Parameter(action_name_ + ".mock_feedback_period", 0.1),
      rclcpp::Parameter(action_name_ + ".mock_initial_distance", 5.0),
    });
    server_node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      unique_test_name("test_nav_server"), options);
    action_server_ = std::make_shared<NavigationFakeActionServer>(
      server_node_.get(), action_name_);

    client_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_nav_client"));
    client_ = rclcpp_action::create_client<Navigation>(client_node_, action_name_);

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
  inline static std::shared_ptr<NavigationFakeActionServer> action_server_;
  inline static rclcpp::Node::SharedPtr client_node_;
  inline static rclcpp_action::Client<Navigation>::SharedPtr client_;
  inline static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  inline static std::thread spin_thread_;
};

TEST_F(NavigationFakeActionServerTest, ExposesConfiguredActionName)
{
  EXPECT_EQ(action_server_->get_action_name(), action_name_);
}

TEST_F(NavigationFakeActionServerTest, RejectsGoalWithEmptyFrameId)
{
  Navigation::Goal goal;
  goal.target_pose = make_pose("");

  auto future = client_->async_send_goal(goal);
  ASSERT_EQ(future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(future.get());
}

TEST_F(NavigationFakeActionServerTest, AcceptsGoalAndSucceedsWithNoError)
{
  Navigation::Goal goal;
  goal.target_pose = make_pose();

  auto goal_handle_future = client_->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  auto wrapped = result_future.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(wrapped.result);
  EXPECT_EQ(wrapped.result->error_code, Navigation::Result::SUCCESS);
}

TEST_F(NavigationFakeActionServerTest, FeedbackReportsDecreasingDistanceRemaining)
{
  Navigation::Goal goal;
  goal.target_pose = make_pose();

  std::mutex mutex;
  std::vector<float> distances;
  rclcpp_action::Client<Navigation>::SendGoalOptions options;
  options.feedback_callback =
    [&mutex, &distances](
    rclcpp_action::ClientGoalHandle<Navigation>::SharedPtr,
    const std::shared_ptr<const Navigation::Feedback> feedback)
    {
      std::lock_guard<std::mutex> lock(mutex);
      distances.push_back(feedback->distance_remaining);
    };

  auto goal_handle_future = client_->async_send_goal(goal, options);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  auto result_future = client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  std::lock_guard<std::mutex> lock(mutex);
  ASSERT_GE(distances.size(), 2u);
  for (std::size_t i = 1; i < distances.size(); ++i) {
    EXPECT_LE(distances[i], distances[i - 1]);
  }
  EXPECT_NEAR(distances.back(), 0.0f, 1e-3f);
}

TEST_F(NavigationFakeActionServerTest, ClientCancelResultsInCanceled)
{
  Navigation::Goal goal;
  goal.target_pose = make_pose();

  auto goal_handle_future = client_->async_send_goal(goal);
  ASSERT_EQ(goal_handle_future.wait_for(2s), std::future_status::ready);
  auto goal_handle = goal_handle_future.get();
  ASSERT_TRUE(goal_handle);

  EXPECT_TRUE(wait_until([this] {return action_server_->is_active();}, 1s));

  auto cancel_future = client_->async_cancel_goal(goal_handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);

  auto result_future = client_->async_get_result(goal_handle);
  ASSERT_EQ(result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(NavigationFakeActionServerTest, NewGoalPreemptsRunningGoalByDefault)
{
  EXPECT_TRUE(action_server_->is_preemptable());

  Navigation::Goal first_goal;
  first_goal.target_pose = make_pose();
  auto first_handle_future = client_->async_send_goal(first_goal);
  ASSERT_EQ(first_handle_future.wait_for(2s), std::future_status::ready);
  auto first_handle = first_handle_future.get();
  ASSERT_TRUE(first_handle);
  EXPECT_TRUE(wait_until([this] {return action_server_->is_active();}, 1s));

  auto first_result_future = client_->async_get_result(first_handle);

  Navigation::Goal second_goal;
  second_goal.target_pose = make_pose();
  auto second_handle_future = client_->async_send_goal(second_goal);
  ASSERT_EQ(second_handle_future.wait_for(2s), std::future_status::ready);
  auto second_handle = second_handle_future.get();
  ASSERT_TRUE(second_handle) << "second goal should be accepted since preemption is allowed";

  ASSERT_EQ(first_result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().code, rclcpp_action::ResultCode::ABORTED);

  auto second_result_future = client_->async_get_result(second_handle);
  ASSERT_EQ(second_result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(second_result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST(NavigationFakeActionServerStandaloneTest, NonPreemptableRejectsSecondGoalWhileBusy)
{
  const auto action_name = unique_test_name("navigation_np");
  rclcpp::NodeOptions options;
  options.parameter_overrides(
  {
    rclcpp::Parameter(action_name + ".allow_preemption", false),
    rclcpp::Parameter(action_name + ".mock_navigation_duration", 1.0),
    rclcpp::Parameter(action_name + ".mock_feedback_period", 0.1),
  });
  auto server_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    unique_test_name("test_nav_server_np"), options);
  auto action_server = std::make_shared<NavigationFakeActionServer>(server_node.get(), action_name);
  ASSERT_FALSE(action_server->is_preemptable());

  auto client_node = std::make_shared<rclcpp::Node>(unique_test_name("test_nav_client_np"));
  auto client = rclcpp_action::create_client<Navigation>(client_node, action_name);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node->get_node_base_interface());
  executor.add_node(client_node);
  auto spin_thread = spin_in_background(executor);

  ASSERT_TRUE(client->wait_for_action_server(5s));

  Navigation::Goal first_goal;
  first_goal.target_pose = make_pose();
  auto first_handle_future = client->async_send_goal(first_goal);
  ASSERT_EQ(first_handle_future.wait_for(2s), std::future_status::ready);
  auto first_handle = first_handle_future.get();
  ASSERT_TRUE(first_handle);
  ASSERT_TRUE(wait_until([&] {return action_server->is_active();}, 1s));

  Navigation::Goal second_goal;
  second_goal.target_pose = make_pose();
  auto second_handle_future = client->async_send_goal(second_goal);
  ASSERT_EQ(second_handle_future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(second_handle_future.get())
    << "second goal should have been rejected while the first is busy";

  auto first_result_future = client->async_get_result(first_handle);
  ASSERT_EQ(first_result_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

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
