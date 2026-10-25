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

#include "easyfleet_mission_manager/robot_handle.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "easyfleet_mission_manager/fleet_session.hpp"

namespace easyfleet
{

RobotHandle::RobotHandle(std::string name)
: name_(std::move(name))
{
}

RobotHandle::~RobotHandle()
{
  if (session_) {
    session_->remove_robot(*this);
  }
}

const std::string & RobotHandle::name() const noexcept
{
  return name_;
}

bool RobotHandle::has_capability(const std::string & capability_type) const
{
  const auto all = capabilities();
  return std::any_of(
    all.begin(), all.end(),
    [&](const easyfleet_mission_manager::CapabilityInfo & info) {
      return info.capability == capability_type && info.active;
    });
}

void RobotHandle::stop_capability(const std::string & capability_type)
{
  const auto it = running_.find(capability_type);
  if (it != running_.end()) {
    it->second->stop();
  }
}

bool RobotHandle::is_capability_running(const std::string & capability_type) const
{
  return capability_state(capability_type) == CapabilityState::RUNNING;
}

CapabilityState RobotHandle::capability_state(const std::string & capability_type) const
{
  const auto it = running_.find(capability_type);
  if (it == running_.end()) {
    return CapabilityState::IDLE;
  }
  return it->second->state();
}

bool RobotHandle::is_alive(const std::string & capability_type) const
{
  return session_ && session_->is_alive(name_, capability_type);
}

void RobotHandle::print_capabilities() const
{
  const auto all = capabilities();
  for (const auto & info : all) {
    easyfleet_mission_manager::print_capability_summary_line(info);
  }
  for (const auto & info : all) {
    easyfleet_mission_manager::safe_print("");
    easyfleet_mission_manager::print_capability_info(info);
  }
}

std::vector<easyfleet_mission_manager::CapabilityInfo> RobotHandle::capabilities() const
{
  if (!session_) {
    return {};
  }
  return session_->capabilities_of(name_);
}

void RobotHandle::attach(FleetSession & session)
{
  session_ = &session;
}

void RobotHandle::detach() noexcept
{
  running_.clear();
  session_ = nullptr;
}

void RobotHandle::check_timeouts()
{
  for (auto & [capability_type, running] : running_) {
    running->check_timeout(is_alive(capability_type));
  }
}

}  // namespace easyfleet
