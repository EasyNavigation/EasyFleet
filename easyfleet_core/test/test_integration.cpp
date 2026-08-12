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

// End-to-end tests that exercise arch_mockup::ActionServerBase and
// arch_mockup::ActionClient together, the way a real application would:
// a node hosting one or more actions, and a separate node holding several
// ActionClient<T>::SharedPtr instances to talk to them.

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "arch_mockup/action_client.hpp"
#include "test_fibonacci_server.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using arch_mockup_test::Fibonacci;
using arch_mockup_test::spin_in_background;
using arch_mockup_test::TestFibonacciServer;
using arch_mockup_test::unique_test_name;
using arch_mockup_test::wait_until;
using FibonacciClient = arch_mockup::ActionClient<Fibonacci>;

// Servers, mission node and the two long-lived clients are shared across the
// whole test suite (rather than recreated per test) to keep the number of
// DDS participants created by this binary low, which otherwise makes
// discovery noticeably slower/flakier in constrained sandboxes.
class IntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    nav_action_name_ = unique_test_name("nav_fibonacci");
    arm_action_name_ = unique_test_name("arm_fibonacci");
    nav_server_ = std::make_shared<TestFibonacciServer>(
      unique_test_name("nav_server"), /*default_allow_preemption=*/true, nav_action_name_);
    arm_server_ = std::make_shared<TestFibonacciServer>(
      unique_test_name("arm_server"), /*default_allow_preemption=*/false, arm_action_name_);
    mission_node_ = std::make_shared<rclcpp::Node>(unique_test_name("mission_node"));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(nav_server_);
    executor_->add_node(arm_server_);
    spin_thread_ = spin_in_background(*executor_);

    nav_client_ = FibonacciClient::create(mission_node_.get(), nav_action_name_);
    arm_client_ = FibonacciClient::create(mission_node_.get(), arm_action_name_);
    ASSERT_TRUE(nav_client_->wait_for_server(5s));
    ASSERT_TRUE(arm_client_->wait_for_server(5s));
  }

  static void TearDownTestSuite()
  {
    nav_client_.reset();
    arm_client_.reset();
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(nav_server_);
    executor_->remove_node(arm_server_);
    executor_.reset();
    mission_node_.reset();
    nav_server_.reset();
    arm_server_.reset();
  }

  void SetUp() override
  {
    nav_server_->set_step_delay(15ms);
    arm_server_->set_step_delay(15ms);
  }

  inline static std::string nav_action_name_;
  inline static std::string arm_action_name_;
  inline static std::shared_ptr<TestFibonacciServer> nav_server_;
  inline static std::shared_ptr<TestFibonacciServer> arm_server_;
  inline static rclcpp::Node::SharedPtr mission_node_;
  inline static FibonacciClient::SharedPtr nav_client_;
  inline static FibonacciClient::SharedPtr arm_client_;
  inline static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  inline static std::thread spin_thread_;
};

TEST_F(IntegrationTest, SeveralActionClientsTalkToTheirOwnServersIndependently)
{
  Fibonacci::Goal goal;
  goal.order = 4;

  auto nav_result = nav_client_->send_goal_and_wait(goal);
  auto arm_result = arm_client_->send_goal_and_wait(goal);

  EXPECT_EQ(nav_result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
  EXPECT_EQ(arm_result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
  ASSERT_TRUE(nav_result.result);
  ASSERT_TRUE(arm_result.result);
  EXPECT_EQ(nav_result.result->sequence.size(), 6u);
  EXPECT_EQ(arm_result.result->sequence.size(), 6u);
}

TEST_F(IntegrationTest, PreemptableActionAbortsRunningGoalForNewOne)
{
  nav_server_->set_step_delay(150ms);

  auto first_result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto first_result_future = first_result_promise->get_future();

  Fibonacci::Goal slow_goal;
  slow_goal.order = 20;
  ASSERT_TRUE(
    nav_client_->send_goal(
      slow_goal, [first_result_promise](const FibonacciClient::GoalResult & result) {
        first_result_promise->set_value(result);
      }));

  EXPECT_TRUE(wait_until([this] {return nav_server_->is_active();}, 2s));

  Fibonacci::Goal fast_goal;
  fast_goal.order = 1;
  auto second_result = nav_client_->send_goal_and_wait(fast_goal);

  ASSERT_EQ(first_result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().outcome, FibonacciClient::GoalOutcome::ABORTED);
  EXPECT_EQ(second_result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
}

TEST_F(IntegrationTest, NonPreemptableActionRejectsNewGoalWhileBusy)
{
  arm_server_->set_step_delay(150ms);
  ASSERT_FALSE(arm_server_->is_preemptable());

  auto first_result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto first_result_future = first_result_promise->get_future();

  Fibonacci::Goal slow_goal;
  slow_goal.order = 10;
  ASSERT_TRUE(
    arm_client_->send_goal(
      slow_goal, [first_result_promise](const FibonacciClient::GoalResult & result) {
        first_result_promise->set_value(result);
      }));

  EXPECT_TRUE(wait_until([this] {return arm_server_->is_active();}, 2s));

  Fibonacci::Goal second_goal;
  second_goal.order = 1;
  auto second_result = arm_client_->send_goal_and_wait(second_goal);
  EXPECT_EQ(second_result.outcome, FibonacciClient::GoalOutcome::REJECTED);

  ASSERT_EQ(first_result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
}

TEST_F(IntegrationTest, TogglingPreemptionParameterAtRuntimeChangesBehaviorEndToEnd)
{
  nav_server_->set_step_delay(150ms);
  ASSERT_TRUE(nav_server_->is_preemptable());

  nav_server_->set_parameter(rclcpp::Parameter(nav_action_name_ + ".allow_preemption", false));
  ASSERT_FALSE(nav_server_->is_preemptable());

  auto first_result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto first_result_future = first_result_promise->get_future();
  Fibonacci::Goal slow_goal;
  slow_goal.order = 10;
  ASSERT_TRUE(
    nav_client_->send_goal(
      slow_goal, [first_result_promise](const FibonacciClient::GoalResult & result) {
        first_result_promise->set_value(result);
      }));
  EXPECT_TRUE(wait_until([this] {return nav_server_->is_active();}, 2s));

  Fibonacci::Goal second_goal;
  second_goal.order = 1;
  auto rejected_result = nav_client_->send_goal_and_wait(second_goal);
  EXPECT_EQ(rejected_result.outcome, FibonacciClient::GoalOutcome::REJECTED);

  nav_server_->set_parameter(rclcpp::Parameter(nav_action_name_ + ".allow_preemption", true));
  ASSERT_TRUE(nav_server_->is_preemptable());

  auto accepted_result = nav_client_->send_goal_and_wait(second_goal);
  EXPECT_EQ(accepted_result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);

  ASSERT_EQ(first_result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(first_result_future.get().outcome, FibonacciClient::GoalOutcome::ABORTED);
}

TEST_F(IntegrationTest, ClientCreatedAndDestroyedInLocalScopeCleansUpPromptly)
{
  const auto start = std::chrono::steady_clock::now();
  {
    auto scoped_client = FibonacciClient::create(mission_node_.get(), nav_action_name_);
    ASSERT_TRUE(scoped_client->wait_for_server(5s));
    Fibonacci::Goal goal;
    goal.order = 2;
    auto result = scoped_client->send_goal_and_wait(goal);
    EXPECT_EQ(result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
  }
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 3s);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
