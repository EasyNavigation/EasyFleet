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
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "test_fibonacci_server.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_core_test::Fibonacci;
using easyfleet_core_test::kOrderForgetToSettle;
using easyfleet_core_test::kOrderThrow;
using easyfleet_core_test::TestFibonacciServer;
using easyfleet_core_test::spin_in_background;
using easyfleet_core_test::unique_test_name;
using easyfleet_core_test::wait_until;

namespace
{

/// Blocking helper around rclcpp_action::Client, built only for these tests
/// so ActionServerBase can be exercised without depending on
/// easyfleet_core::ActionClient (that class has its own dedicated test suite).
struct SyncGoalOutcome
{
  bool accepted{false};
  rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
  Fibonacci::Result::SharedPtr result;
  int feedback_count{0};
};

}  // namespace

// The server, client and executor are shared across the whole test suite
// (rather than recreated per test) to keep the number of DDS participants
// created by this binary low, which otherwise makes discovery noticeably
// slower/flakier in constrained sandboxes.
class ActionServerBaseTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    action_name_ = unique_test_name("fibonacci");
    server_node_ = std::make_shared<TestFibonacciServer>(
      unique_test_name("test_fibonacci_server"), true, action_name_);
    client_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_fibonacci_client"));
    client_ = rclcpp_action::create_client<Fibonacci>(client_node_, action_name_);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node_);
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
    executor_->remove_node(server_node_);
    executor_->remove_node(client_node_);
    executor_.reset();
    client_.reset();
    client_node_.reset();
    server_node_.reset();
  }

  void SetUp() override
  {
    server_node_->set_step_delay(15ms);
  }

  // Sends a goal and blocks until it reaches a terminal state (or the goal
  // is rejected). `on_feedback` is invoked for every feedback message.
  SyncGoalOutcome send_and_wait(
    int32_t order,
    std::function<void(const Fibonacci::Feedback &)> on_feedback = nullptr)
  {
    SyncGoalOutcome outcome;
    auto result_promise = std::make_shared<std::promise<void>>();
    auto result_future = result_promise->get_future();

    rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
    options.feedback_callback =
      [&outcome, on_feedback](
      rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr,
      const std::shared_ptr<const Fibonacci::Feedback> feedback)
      {
        outcome.feedback_count++;
        if (on_feedback) {
          on_feedback(*feedback);
        }
      };
    options.result_callback =
      [&outcome, result_promise](
      const rclcpp_action::ClientGoalHandle<Fibonacci>::WrappedResult & wrapped)
      {
        outcome.code = wrapped.code;
        outcome.result = wrapped.result;
        result_promise->set_value();
      };

    Fibonacci::Goal goal;
    goal.order = order;
    auto goal_handle_future = client_->async_send_goal(goal, options);

    if (goal_handle_future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "Timed out waiting for goal response";
      return outcome;
    }
    auto goal_handle = goal_handle_future.get();
    outcome.accepted = (goal_handle != nullptr);
    if (!outcome.accepted) {
      return outcome;
    }

    if (result_future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "Timed out waiting for goal result";
    }
    return outcome;
  }

  // Sends a goal and returns immediately with its handle (once accepted).
  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr send_async(
    int32_t order,
    rclcpp_action::Client<Fibonacci>::ResultCallback result_cb = nullptr)
  {
    rclcpp_action::Client<Fibonacci>::SendGoalOptions options;
    options.result_callback = result_cb;
    Fibonacci::Goal goal;
    goal.order = order;
    auto future = client_->async_send_goal(goal, options);
    if (future.wait_for(5s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  inline static std::string action_name_;
  inline static std::shared_ptr<TestFibonacciServer> server_node_;
  inline static rclcpp::Node::SharedPtr client_node_;
  inline static rclcpp_action::Client<Fibonacci>::SharedPtr client_;
  inline static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  inline static std::thread spin_thread_;
};

TEST_F(ActionServerBaseTest, AcceptsValidGoalAndSucceeds)
{
  auto outcome = send_and_wait(5);
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(outcome.result);
  EXPECT_EQ(outcome.result->sequence.size(), 7u);  // seed(0,1) + 5 steps
  EXPECT_GT(outcome.feedback_count, 0);
  EXPECT_FALSE(server_node_->is_active());
}

TEST_F(ActionServerBaseTest, RejectsInvalidGoal)
{
  const auto goals_started_before = server_node_->goals_started();
  auto outcome = send_and_wait(-5);
  EXPECT_FALSE(outcome.accepted);
  EXPECT_EQ(server_node_->goals_started(), goals_started_before);
}

TEST_F(ActionServerBaseTest, IsActiveReflectsExecutionState)
{
  EXPECT_FALSE(server_node_->is_active());
  server_node_->set_step_delay(200ms);

  auto handle = send_async(3);
  ASSERT_TRUE(handle);

  EXPECT_TRUE(wait_until([this] {return server_node_->is_active();}, 2s));

  client_->async_cancel_goal(handle);
  EXPECT_TRUE(wait_until([this] {return !server_node_->is_active();}, 2s));
}

TEST_F(ActionServerBaseTest, DefaultsToPreemptable)
{
  EXPECT_TRUE(server_node_->is_preemptable());
}

TEST_F(ActionServerBaseTest, PreemptionDisallowed_SecondGoalRejectedWhileBusy)
{
  const auto np_action_name = unique_test_name("fibonacci_np");
  auto not_preemptable_server = std::make_shared<TestFibonacciServer>(
    unique_test_name("server_np"), /*default_allow_preemption=*/false, np_action_name);
  not_preemptable_server->set_step_delay(150ms);
  executor_->add_node(not_preemptable_server);

  auto client_node = std::make_shared<rclcpp::Node>(unique_test_name("client_np"));
  auto client = rclcpp_action::create_client<Fibonacci>(client_node, np_action_name);
  executor_->add_node(client_node);
  ASSERT_TRUE(client->wait_for_action_server(5s));

  EXPECT_FALSE(not_preemptable_server->is_preemptable());

  Fibonacci::Goal goal1;
  goal1.order = 5;
  auto handle1_future = client->async_send_goal(goal1);
  ASSERT_EQ(handle1_future.wait_for(2s), std::future_status::ready);
  auto handle1 = handle1_future.get();
  ASSERT_TRUE(handle1);

  EXPECT_TRUE(wait_until([&] {return not_preemptable_server->is_active();}, 2s));

  Fibonacci::Goal goal2;
  goal2.order = 2;
  auto handle2_future = client->async_send_goal(goal2);
  ASSERT_EQ(handle2_future.wait_for(2s), std::future_status::ready);
  auto handle2 = handle2_future.get();
  EXPECT_FALSE(handle2) << "second goal should have been rejected while the first is busy";

  auto result_future = client->async_get_result(handle1);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  executor_->remove_node(not_preemptable_server);
  executor_->remove_node(client_node);
}

TEST_F(ActionServerBaseTest, PreemptionAllowed_SecondGoalPreemptsFirst)
{
  server_node_->set_step_delay(150ms);

  Fibonacci::Goal goal1;
  goal1.order = 20;
  auto handle1_future = client_->async_send_goal(goal1);
  ASSERT_EQ(handle1_future.wait_for(2s), std::future_status::ready);
  auto handle1 = handle1_future.get();
  ASSERT_TRUE(handle1);

  EXPECT_TRUE(wait_until([this] {return server_node_->is_active();}, 2s));

  auto result1_future = client_->async_get_result(handle1);

  Fibonacci::Goal goal2;
  goal2.order = 2;
  auto handle2_future = client_->async_send_goal(goal2);
  ASSERT_EQ(handle2_future.wait_for(2s), std::future_status::ready);
  auto handle2 = handle2_future.get();
  ASSERT_TRUE(handle2) << "second goal should be accepted since preemption is allowed";

  ASSERT_EQ(result1_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result1_future.get().code, rclcpp_action::ResultCode::ABORTED);

  auto result2_future = client_->async_get_result(handle2);
  ASSERT_EQ(result2_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result2_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);

  EXPECT_GE(server_node_->goals_preempted(), 1);
  EXPECT_GE(server_node_->on_preempted_calls(), 1);
}

TEST_F(ActionServerBaseTest, DynamicParameterChange_TogglesPreemptionImmediately)
{
  ASSERT_TRUE(server_node_->is_preemptable());
  server_node_->set_parameter(rclcpp::Parameter(action_name_ + ".allow_preemption", false));
  EXPECT_FALSE(server_node_->is_preemptable());

  server_node_->set_step_delay(150ms);
  Fibonacci::Goal goal1;
  goal1.order = 10;
  auto handle1_future = client_->async_send_goal(goal1);
  ASSERT_EQ(handle1_future.wait_for(2s), std::future_status::ready);
  auto handle1 = handle1_future.get();
  ASSERT_TRUE(handle1);
  EXPECT_TRUE(wait_until([this] {return server_node_->is_active();}, 2s));

  Fibonacci::Goal goal2;
  goal2.order = 1;
  auto handle2_future = client_->async_send_goal(goal2);
  ASSERT_EQ(handle2_future.wait_for(2s), std::future_status::ready);
  EXPECT_FALSE(handle2_future.get()) << "preemption was disabled at runtime";

  server_node_->set_parameter(rclcpp::Parameter(action_name_ + ".allow_preemption", true));
  EXPECT_TRUE(server_node_->is_preemptable());

  auto result1_future = client_->async_get_result(handle1);
  ASSERT_EQ(result1_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result1_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(ActionServerBaseTest, QueuedGoalSupersededBeforeStarting_IsAborted)
{
  server_node_->set_step_delay(150ms);

  Fibonacci::Goal goal1;
  goal1.order = 20;
  auto handle1 = send_async(20);
  ASSERT_TRUE(handle1);
  EXPECT_TRUE(wait_until([this] {return server_node_->is_active();}, 2s));

  // goal2 becomes "pending" (goal1 is still running).
  Fibonacci::Goal goal2;
  goal2.order = 3;
  auto handle2_future = client_->async_send_goal(goal2);
  ASSERT_EQ(handle2_future.wait_for(2s), std::future_status::ready);
  auto handle2 = handle2_future.get();
  ASSERT_TRUE(handle2);
  auto result2_future = client_->async_get_result(handle2);

  // goal3 arrives immediately after: it should bump goal2 out of the queue.
  Fibonacci::Goal goal3;
  goal3.order = 1;
  auto handle3_future = client_->async_send_goal(goal3);
  ASSERT_EQ(handle3_future.wait_for(2s), std::future_status::ready);
  auto handle3 = handle3_future.get();
  ASSERT_TRUE(handle3);

  ASSERT_EQ(result2_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result2_future.get().code, rclcpp_action::ResultCode::ABORTED);

  auto result3_future = client_->async_get_result(handle3);
  ASSERT_EQ(result3_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result3_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(ActionServerBaseTest, ClientCancelRequest_ResultsInCanceled)
{
  server_node_->set_step_delay(150ms);
  auto handle = send_async(20);
  ASSERT_TRUE(handle);
  EXPECT_TRUE(wait_until([this] {return server_node_->is_active();}, 2s));

  auto cancel_future = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel_future.wait_for(2s), std::future_status::ready);

  auto result_future = client_->async_get_result(handle);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(wait_until([this] {return !server_node_->is_active();}, 2s));
}

TEST_F(ActionServerBaseTest, ExceptionDuringExecute_ResultsInAborted)
{
  auto outcome = send_and_wait(kOrderThrow);
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(ActionServerBaseTest, ForgettingToSettleGoal_IsAutoAborted)
{
  auto outcome = send_and_wait(kOrderForgetToSettle);
  ASSERT_TRUE(outcome.accepted);
  EXPECT_EQ(outcome.code, rclcpp_action::ResultCode::ABORTED);
}

TEST_F(ActionServerBaseTest, SequentialGoalsRunOneAfterAnother)
{
  const auto goals_started_before = server_node_->goals_started();

  auto outcome1 = send_and_wait(2);
  ASSERT_TRUE(outcome1.accepted);
  EXPECT_EQ(outcome1.code, rclcpp_action::ResultCode::SUCCEEDED);

  auto outcome2 = send_and_wait(3);
  ASSERT_TRUE(outcome2.accepted);
  EXPECT_EQ(outcome2.code, rclcpp_action::ResultCode::SUCCEEDED);

  EXPECT_EQ(server_node_->goals_started(), goals_started_before + 2);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
