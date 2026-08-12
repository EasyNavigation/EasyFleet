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
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "arch_mockup/capability_client.hpp"
#include "test_fibonacci_server.hpp"
#include "test_utils.hpp"

using namespace std::chrono_literals;
using arch_mockup_test::Fibonacci;
using arch_mockup_test::spin_in_background;
using arch_mockup_test::TestFibonacciServer;
using arch_mockup_test::unique_test_name;
using arch_mockup_test::wait_until;
using FibonacciCapabilityClient = arch_mockup::CapabilityClient<Fibonacci>;

// The server and its owning executor are shared across the whole test suite
// (rather than recreated per test) to keep the number of DDS participants
// created by this binary low.
class CapabilityClientTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    action_name_ = unique_test_name("fibonacci");
    server_node_ = std::make_shared<TestFibonacciServer>(
      unique_test_name("test_fibonacci_server_for_capability_client"), true, action_name_);
    owner_node_ = std::make_shared<rclcpp::Node>(unique_test_name("test_capability_client_owner"));

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

TEST_F(CapabilityClientTest, WaitForCapabilitySucceedsWhenAvailable)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  EXPECT_TRUE(client->wait_for_capability(5s));
}

TEST_F(CapabilityClientTest, WaitForCapabilityTimesOutForUnknownCapability)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), "no_such_capability");
  const auto start = std::chrono::steady_clock::now();
  EXPECT_FALSE(client->wait_for_capability(300ms));
  EXPECT_LT(std::chrono::steady_clock::now() - start, 2s);
}

TEST_F(CapabilityClientTest, RequestInvokesFeedbackAndResponseCallbacks)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  ASSERT_TRUE(client->wait_for_capability(5s));

  std::atomic<int> feedback_count{0};
  auto response_promise = std::make_shared<std::promise<FibonacciCapabilityClient::Response>>();
  auto response_future = response_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = 5;
  client->request(
    goal,
    [response_promise](const FibonacciCapabilityClient::Response & response) {
      response_promise->set_value(response);
    },
    [&feedback_count](std::shared_ptr<const Fibonacci::Feedback>) {
      feedback_count.fetch_add(1);
    });

  ASSERT_EQ(response_future.wait_for(5s), std::future_status::ready);
  auto response = response_future.get();

  EXPECT_GT(feedback_count.load(), 0);
  EXPECT_EQ(response.outcome, FibonacciCapabilityClient::Outcome::SUCCEEDED);
  ASSERT_TRUE(response.result);
  EXPECT_EQ(response.result->sequence.size(), 7u);
}

TEST_F(CapabilityClientTest, RequestRejectedGoalReportsRejectedOutcome)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  ASSERT_TRUE(client->wait_for_capability(5s));

  auto response_promise = std::make_shared<std::promise<FibonacciCapabilityClient::Response>>();
  auto response_future = response_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = -5;
  client->request(
    goal, [response_promise](const FibonacciCapabilityClient::Response & response) {
      response_promise->set_value(response);
    });

  ASSERT_EQ(response_future.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(response_future.get().outcome, FibonacciCapabilityClient::Outcome::REJECTED);
}

TEST_F(CapabilityClientTest, RequestAndWaitSucceeds)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  ASSERT_TRUE(client->wait_for_capability(5s));

  Fibonacci::Goal goal;
  goal.order = 4;
  auto response = client->request_and_wait(goal);

  EXPECT_EQ(response.outcome, FibonacciCapabilityClient::Outcome::SUCCEEDED);
  ASSERT_TRUE(response.result);
  EXPECT_EQ(response.result->sequence.size(), 6u);
}

TEST_F(CapabilityClientTest, RequestAndWaitReturnsImmediatelyWhenCapabilityUnavailable)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), "no_such_capability", 200ms);

  Fibonacci::Goal goal;
  goal.order = 1;
  const auto start = std::chrono::steady_clock::now();
  auto response = client->request_and_wait(goal);
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(response.outcome, FibonacciCapabilityClient::Outcome::SERVER_UNAVAILABLE);
  EXPECT_LT(elapsed, 1s);
}

TEST_F(CapabilityClientTest, RequestAndWaitTimesOutOnSlowGoal)
{
  server_node_->set_step_delay(300ms);
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  ASSERT_TRUE(client->wait_for_capability(5s));

  Fibonacci::Goal goal;
  goal.order = 20;
  auto response = client->request_and_wait(goal, nullptr, 150ms);

  EXPECT_EQ(response.outcome, FibonacciCapabilityClient::Outcome::TIMEOUT);
}

TEST_F(CapabilityClientTest, FeedbackCallbackIsInvokedDuringRequestAndWait)
{
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  ASSERT_TRUE(client->wait_for_capability(5s));

  std::atomic<int> feedback_count{0};
  Fibonacci::Goal goal;
  goal.order = 4;
  auto response = client->request_and_wait(
    goal, [&feedback_count](std::shared_ptr<const Fibonacci::Feedback>) {
      feedback_count.fetch_add(1);
    });

  EXPECT_EQ(response.outcome, FibonacciCapabilityClient::Outcome::SUCCEEDED);
  EXPECT_GT(feedback_count.load(), 0);
}

TEST_F(CapabilityClientTest, CancelStopsAnInFlightRequest)
{
  server_node_->set_step_delay(150ms);
  auto client = FibonacciCapabilityClient::create(owner_node_.get(), action_name_);
  ASSERT_TRUE(client->wait_for_capability(5s));

  auto response_promise = std::make_shared<std::promise<FibonacciCapabilityClient::Response>>();
  auto response_future = response_promise->get_future();

  Fibonacci::Goal goal;
  goal.order = 20;
  client->request(
    goal, [response_promise](const FibonacciCapabilityClient::Response & response) {
      response_promise->set_value(response);
    });

  ASSERT_TRUE(wait_until([this] {return server_node_->is_active();}, 2s));

  client->cancel();

  ASSERT_EQ(response_future.wait_for(3s), std::future_status::ready);
  EXPECT_EQ(response_future.get().outcome, FibonacciCapabilityClient::Outcome::CANCELED);
  EXPECT_TRUE(wait_until([this] {return !server_node_->is_active();}, 2s));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
