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

#ifndef EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__TEST__TEST_UTILS_HPP_
#define EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__TEST__TEST_UTILS_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

#include "rclcpp/executor.hpp"

namespace easyfleet_fake_collaboration_deployment_test
{

/// Appends a process-wide unique suffix to `base`, so tests that create
/// their own throwaway nodes/actions never collide with each other.
inline std::string unique_test_name(const std::string & base)
{
  static std::atomic<uint64_t> counter{0};
  return base + "_" + std::to_string(counter.fetch_add(1));
}

/// Polls `pred` until it returns true or `timeout` elapses. Returns the last
/// value of `pred`. Used instead of fixed sleeps to keep tests fast and
/// non-flaky.
template<typename Predicate>
bool wait_until(
  Predicate pred,
  std::chrono::milliseconds timeout,
  std::chrono::milliseconds poll = std::chrono::milliseconds(10))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(poll);
  }
  return pred();
}

/// Starts `executor.spin()` on a background thread and blocks until it has
/// actually begun spinning before returning it. `Executor::cancel()` only
/// reliably interrupts a `spin()` that has already started: calling it any
/// earlier races with the thread below and can block `join()` forever.
inline std::thread spin_in_background(rclcpp::Executor & executor)
{
  std::thread thread([&executor] {executor.spin();});
  while (!executor.is_spinning()) {
    std::this_thread::yield();
  }
  return thread;
}

}  // namespace easyfleet_fake_collaboration_deployment_test

#endif  // EASYFLEET_FAKE_COLLABORATION_DEPLOYMENT__TEST__TEST_UTILS_HPP_
