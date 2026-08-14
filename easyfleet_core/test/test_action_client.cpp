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
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "easyfleet_core/action_client.hpp"
#include "test_fibonacci_server.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using easyfleet_core_test::Fibonacci;
using easyfleet_core_test::spin_in_background;
using easyfleet_core_test::TestFibonacciServer;
using easyfleet_core_test::unique_test_name;
using easyfleet_core_test::wait_until;
using FibonacciClient = easyfleet_core::ActionClient<Fibonacci>;

// The server and its owning executor are shared across the whole test suite
// (rather than recreated per test) to keep the number of DDS participants
// created by this binary low: each ActionClient still gets its own fresh
// internal node per test, so client-level behavior stays fully isolated.
class ActionClientTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    action_name_ = unique_test_name("fibonacci");
    server_node_ = std::make_shared<TestFibonacciServer>(
      unique_test_name("test_fibonacci_server_for_client"), true, action_name_);
    owner_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_client_owner"));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node_);
    spin_thread_ = spin_in_background(*executor_);
  }

  static void TearDownTestSuite()
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(server_node_);
    executor_.reset();
    owner_node_.reset();
    server_node_.reset();
  }

  void SetUp() override
  {
    server_node_->set_step_delay(15ms);
  }

  inline static std::string action_name_;
  inline static std::shared_ptr<TestFibonacciServer> server_node_;
  inline static rclcpp::Node::SharedPtr owner_node_;
  inline static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  inline static std::thread spin_thread_;
};

TEST_F(ActionClientTest, WaitForServerSucceedsWhenServerIsUp)
{
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  EXPECT_TRUE(client->wait_for_server(5s));
}

TEST_F(ActionClientTest, WaitForServerTimesOutForUnknownAction)
{
  auto client = FibonacciClient::create(*owner_node_, "no_such_action");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client->wait_for_server(300ms));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 2s);
}

TEST_F(ActionClientTest, SendGoalAsyncInvokesAllCallbacks)
{
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));

  std::atomic<bool> response_called{false};
  std::atomic<bool> response_accepted{false};
  std::atomic<int> feedback_count{0};
  auto result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto result_future = result_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = 5;
  bool dispatched = client->send_goal(
    goal,
    [result_promise](const FibonacciClient::GoalResult & result) {
      result_promise->set_value(result);
    },
    [&feedback_count](std::shared_ptr<const Fibonacci::Feedback>) {
      feedback_count.fetch_add(1);
    },
    [&response_called, &response_accepted](bool accepted, const rclcpp_action::GoalUUID &) {
      response_called.store(true);
      response_accepted.store(accepted);
    });

  ASSERT_TRUE(dispatched);
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  auto result = result_future.get();

  EXPECT_TRUE(response_called.load());
  EXPECT_TRUE(response_accepted.load());
  EXPECT_GT(feedback_count.load(), 0);
  EXPECT_EQ(result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->sequence.size(), 7u);
}

TEST_F(ActionClientTest, SendGoalAsyncRejectedGoalReportsRejected)
{
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));

  auto result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto result_future = result_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = -5;
  client->send_goal(
    goal, [result_promise](const FibonacciClient::GoalResult & result) {
      result_promise->set_value(result);
    });

  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_future.get().outcome, FibonacciClient::GoalOutcome::REJECTED);
}

TEST_F(ActionClientTest, SendGoalAndWaitSucceeds)
{
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));

  Fibonacci::Goal goal;
  goal.order = 4;
  auto result = client->send_goal_and_wait(goal);

  EXPECT_EQ(result.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
  ASSERT_TRUE(result.result);
  EXPECT_EQ(result.result->sequence.size(), 6u);
}

TEST_F(ActionClientTest, SendGoalAndWaitServerUnavailableReturnsImmediately)
{
  auto client = FibonacciClient::create(*owner_node_, "no_such_action", 200ms);

  Fibonacci::Goal goal;
  goal.order = 1;
  const auto start = std::chrono::steady_clock::now();
  auto result = client->send_goal_and_wait(goal);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(result.outcome, FibonacciClient::GoalOutcome::SERVER_UNAVAILABLE);
  EXPECT_LT(elapsed, 1s);
}

TEST_F(ActionClientTest, SendGoalAndWaitTimesOutOnSlowGoal)
{
  server_node_->set_step_delay(300ms);
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));

  Fibonacci::Goal goal;
  goal.order = 20;
  auto result = client->send_goal_and_wait(goal, nullptr, 150ms);

  EXPECT_EQ(result.outcome, FibonacciClient::GoalOutcome::TIMEOUT);
}

TEST_F(ActionClientTest, CancelGoalByIdResultsInCanceled)
{
  server_node_->set_step_delay(150ms);
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));

  auto id_promise = std::make_shared<std::promise<rclcpp_action::GoalUUID>>();
  auto id_future = id_promise->get_future();
  auto result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto result_future = result_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = 20;
  client->send_goal(
    goal,
    [result_promise](const FibonacciClient::GoalResult & result) {
      result_promise->set_value(result);
    },
    nullptr,
    [id_promise](bool accepted, const rclcpp_action::GoalUUID & goal_id) {
      if (accepted) {
        id_promise->set_value(goal_id);
      }
    });

  ASSERT_EQ(id_future.wait_for(5s), std::future_status::ready);
  auto goal_id = id_future.get();

  EXPECT_TRUE(wait_until([&] {return client->active_goal_count() == 1u;}, 2s));
  EXPECT_TRUE(client->cancel_goal(goal_id));

  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_future.get().outcome, FibonacciClient::GoalOutcome::CANCELED);
  EXPECT_TRUE(wait_until([&] {return client->active_goal_count() == 0u;}, 2s));
}

TEST_F(ActionClientTest, CancelGoalWithUnknownIdReturnsFalse)
{
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));
  EXPECT_FALSE(client->cancel_goal(rclcpp_action::GoalUUID{}));
}

TEST_F(ActionClientTest, CancelAllGoalsResultsInCanceled)
{
  server_node_->set_step_delay(150ms);
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));

  auto result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto result_future = result_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = 20;
  client->send_goal(
    goal, [result_promise](const FibonacciClient::GoalResult & result) {
      result_promise->set_value(result);
    });

  EXPECT_TRUE(wait_until([&] {return client->active_goal_count() == 1u;}, 2s));
  client->cancel_all_goals();

  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(result_future.get().outcome, FibonacciClient::GoalOutcome::CANCELED);
}

TEST_F(ActionClientTest, ActiveGoalCountTracksLifecycle)
{
  auto client = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client->wait_for_server(5s));
  EXPECT_EQ(client->active_goal_count(), 0u);

  auto result_promise = std::make_shared<std::promise<FibonacciClient::GoalResult>>();
  auto result_future = result_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = 3;
  client->send_goal(
    goal, [result_promise](const FibonacciClient::GoalResult & result) {
      result_promise->set_value(result);
    });

  EXPECT_TRUE(wait_until([&] {return client->active_goal_count() == 1u;}, 2s));
  ASSERT_EQ(result_future.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(wait_until([&] {return client->active_goal_count() == 0u;}, 2s));
}

TEST_F(ActionClientTest, MultipleIndependentClientsCanTalkToSameServer)
{
  auto client_a = FibonacciClient::create(*owner_node_, action_name_);
  auto client_b = FibonacciClient::create(*owner_node_, action_name_);
  ASSERT_TRUE(client_a->wait_for_server(5s));
  ASSERT_TRUE(client_b->wait_for_server(5s));

  Fibonacci::Goal goal;
  goal.order = 2;
  auto result_a = client_a->send_goal_and_wait(goal);
  auto result_b = client_b->send_goal_and_wait(goal);

  EXPECT_EQ(result_a.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
  EXPECT_EQ(result_b.outcome, FibonacciClient::GoalOutcome::SUCCEEDED);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
