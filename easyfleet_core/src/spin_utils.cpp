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

#include "easyfleet_core/spin_utils.hpp"

#include <csignal>
#include <functional>

namespace easyfleet_core
{

namespace
{

// std::signal only accepts a plain function pointer, so the executor to
// cancel has to be reached through a global -- safe here because this
// project never calls spin_until_shutdown() more than once concurrently
// within a single process.
std::function<void()> * g_cancel_callback = nullptr;

void handle_signal(int /*signal_number*/)
{
  if (g_cancel_callback) {
    (*g_cancel_callback)();
  }
}

}  // namespace

void spin_until_shutdown(rclcpp::Executor & executor)
{
  std::function<void()> cancel = [&executor] {executor.cancel();};
  g_cancel_callback = &cancel;

  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);

  executor.spin();

  g_cancel_callback = nullptr;
  std::signal(SIGINT, SIG_DFL);
  std::signal(SIGTERM, SIG_DFL);
}

}  // namespace easyfleet_core
