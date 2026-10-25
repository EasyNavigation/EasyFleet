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

// A server process that dies abruptly (SIGKILL) while a goal is in flight
// never sends a result. A server destroyed cleanly aborts its goals itself,
// so the crash has to be a real one, in a child process forked before any
// ROS initialization.

#include <gtest/gtest.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "easyfleet_core/action_client.hpp"
#include "example_interfaces/action/fibonacci.hpp"

using namespace std::chrono_literals;
using Fibonacci = example_interfaces::action::Fibonacci;
using FibonacciClient = easyfleet_core::ActionClient<Fibonacci>;

namespace
{

std::string g_action_name;  // NOLINT
pid_t g_server_pid = -1;

/// Child process: accepts every goal and never settles it, until killed.
[[noreturn]] void run_silent_server(const std::string & action_name)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("silent_server");
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>> handles;
  auto server = rclcpp_action::create_server<Fibonacci>(
    node, action_name,
    [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&handles](std::shared_ptr<rclcpp_action::ServerGoalHandle<Fibonacci>> handle) {
      handles.push_back(handle);
    });
  rclcpp::spin(node);
  std::_Exit(0);
}

}  // namespace

TEST(ActionClientServerLossTest, SendGoalAndWaitReturnsServerUnavailableWhenTheServerIsKilled)
{
  auto owner = std::make_shared<rclcpp::Node>("server_loss_owner");
  auto client = FibonacciClient::create(*owner, g_action_name, 1s);
  ASSERT_TRUE(client->wait_for_server(20s));

  std::thread killer([client] {
      const auto deadline = std::chrono::steady_clock::now() + 10s;
      while (client->active_goal_count() == 0u && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(10ms);
      }
      std::this_thread::sleep_for(200ms);
      kill(g_server_pid, SIGKILL);
    });

  Fibonacci::Goal goal;
  goal.order = 1;
  auto result = client->send_goal_and_wait(goal);  // no timeout: must not hang
  killer.join();

  EXPECT_EQ(result.outcome, FibonacciClient::GoalOutcome::SERVER_UNAVAILABLE);
}

int main(int argc, char ** argv)
{
  g_action_name = "silent_fibonacci_" + std::to_string(::getpid());
  g_server_pid = fork();
  if (g_server_pid == 0) {
    run_silent_server(g_action_name);
  }

  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  kill(g_server_pid, SIGKILL);
  waitpid(g_server_pid, nullptr, 0);
  return result;
}
