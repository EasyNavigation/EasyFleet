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

#ifndef EASYFLEET_MISSION_MANAGER__RUN_CAPABILITY_HPP_
#define EASYFLEET_MISSION_MANAGER__RUN_CAPABILITY_HPP_

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "easyfleet_core/capability_client.hpp"

#include "easyfleet_mission_manager/ansi.hpp"
#include "easyfleet_mission_manager/output.hpp"

namespace easyfleet_mission_manager
{

inline std::string outcome_to_string(uint8_t outcome_value)
{
  // Mirrors easyfleet_core::CapabilityClient<ActionT>::Outcome's underlying values.
  switch (outcome_value) {
    case 0: return "SUCCEEDED";
    case 1: return "ABORTED";
    case 2: return "CANCELED";
    case 3: return "REJECTED";
    case 4: return "TIMEOUT";
    case 5: return "SERVER_UNAVAILABLE";
    default: return "UNKNOWN";
  }
}

/// @brief Sends `goal` to a capability through `client`, printing progress as it
/// happens: a start line, live feedback (via `on_feedback`), and either the
/// natural terminal outcome or -- if `timeout` elapses first -- a message
/// that the capability is being stopped, followed by the outcome of that
/// cancellation. Blocks the calling thread until the capability is done.
template<typename ActionT>
typename easyfleet_core::CapabilityClient<ActionT>::Response run_capability(
  typename easyfleet_core::CapabilityClient<ActionT>::SharedPtr client,
  const std::string & label,
  const typename ActionT::Goal & goal,
  std::chrono::seconds timeout,
  std::function<void(const typename ActionT::Feedback &)> on_feedback)
{
  using Client = easyfleet_core::CapabilityClient<ActionT>;
  using Feedback = typename ActionT::Feedback;

  auto response_promise = std::make_shared<std::promise<typename Client::Response>>();
  auto response_future = response_promise->get_future();

  {
    std::ostringstream out;
    out << ansi::bold << ansi::cyan << "[" << label << "] " << ansi::reset
        << "sending goal (will run for up to " << timeout.count() << "s)...";
    safe_print(out.str());
  }

  client->request(
    goal,
    [response_promise](const typename Client::Response & response) {
      response_promise->set_value(response);
    },
    [on_feedback, label](std::shared_ptr<const Feedback> feedback) {
      if (on_feedback) {
        on_feedback(*feedback);
      }
    });

  typename Client::Response response;
  if (response_future.wait_for(timeout) == std::future_status::timeout) {
    std::ostringstream out;
    out << ansi::yellow << "[" << label << "] " << ansi::reset
        << timeout.count() << "s elapsed, stopping it...";
    safe_print(out.str());
    client->cancel();
    response = response_future.get();
  } else {
    response = response_future.get();
  }

  std::ostringstream out;
  out << ansi::bold << "[" << label << "] " << ansi::reset
      << "finished with outcome " << ansi::magenta
      << outcome_to_string(static_cast<uint8_t>(response.outcome)) << ansi::reset;
  safe_print(out.str());

  return response;
}

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__RUN_CAPABILITY_HPP_
