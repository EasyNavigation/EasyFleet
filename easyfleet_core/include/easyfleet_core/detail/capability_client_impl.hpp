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

#ifndef EASYFLEET_CORE__DETAIL__CAPABILITY_CLIENT_IMPL_HPP_
#define EASYFLEET_CORE__DETAIL__CAPABILITY_CLIENT_IMPL_HPP_

// Out-of-line member definitions for easyfleet_core::CapabilityClient<ActionT>.
// Included from the bottom of easyfleet_core/capability_client.hpp. Not meant
// to be included directly: templates cannot be compiled into easyfleet_core's
// .cpp/.so, so the implementation lives here to keep the class declaration
// in capability_client.hpp free of member bodies.

#include <memory>
#include <string>
#include <utility>

namespace easyfleet_core
{

template<typename ActionT>
typename CapabilityClient<ActionT>::SharedPtr CapabilityClient<ActionT>::create(
  rclcpp::Node & node,
  const std::string & capability_name,
  std::chrono::milliseconds default_wait_timeout)
{
  return SharedPtr(
    new CapabilityClient(
      ActionClient<ActionT>::create(node, capability_name, default_wait_timeout)));
}

template<typename ActionT>
CapabilityClient<ActionT>::CapabilityClient(typename ActionClient<ActionT>::SharedPtr action_client)
: action_client_(std::move(action_client))
{
}

template<typename ActionT>
bool CapabilityClient<ActionT>::wait_for_capability()
{
  return action_client_->wait_for_server();
}

template<typename ActionT>
bool CapabilityClient<ActionT>::wait_for_capability(std::chrono::milliseconds timeout)
{
  return action_client_->wait_for_server(timeout);
}

template<typename ActionT>
void CapabilityClient<ActionT>::request(
  const Goal & goal,
  ResponseCallback on_response,
  FeedbackCallback on_feedback)
{
  action_client_->send_goal(
    goal,
    [on_response](const typename ActionClient<ActionT>::GoalResult & result) {
      if (on_response) {
        on_response(Response{result.outcome, result.result});
      }
    },
    on_feedback);
}

template<typename ActionT>
typename CapabilityClient<ActionT>::Response CapabilityClient<ActionT>::request_and_wait(
  const Goal & goal,
  FeedbackCallback on_feedback,
  std::chrono::milliseconds timeout)
{
  const auto result = action_client_->send_goal_and_wait(goal, on_feedback, timeout);
  return Response{result.outcome, result.result};
}

template<typename ActionT>
void CapabilityClient<ActionT>::cancel()
{
  action_client_->cancel_all_goals();
}

}  // namespace easyfleet_core

#endif  // EASYFLEET_CORE__DETAIL__CAPABILITY_CLIENT_IMPL_HPP_
