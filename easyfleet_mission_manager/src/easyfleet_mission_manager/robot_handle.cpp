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

namespace
{

/// How long without a heartbeat before is_alive() gives up on a
/// capability -- a few times the 1 Hz heartbeat period Capability<T>
/// itself publishes at, so one or two dropped messages don't flip it.
constexpr std::chrono::seconds kAliveTimeout(3);

}  // namespace

RobotHandle::RobotHandle(std::string name)
: name_(std::move(name))
{
}

const std::string & RobotHandle::name() const noexcept
{
  return name_;
}

bool RobotHandle::has_capability(const std::string & capability_type) const
{
  return std::any_of(
    capabilities_.begin(), capabilities_.end(),
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
  const auto it = last_heartbeat_.find(capability_type);
  if (it == last_heartbeat_.end()) {
    return false;
  }
  return std::chrono::steady_clock::now() - it->second < kAliveTimeout;
}

void RobotHandle::print_capabilities() const
{
  for (const auto & info : capabilities_) {
    easyfleet_mission_manager::print_capability_summary_line(info);
  }
  for (const auto & info : capabilities_) {
    easyfleet_mission_manager::safe_print("");
    easyfleet_mission_manager::print_capability_info(info);
  }
}

const std::vector<easyfleet_mission_manager::CapabilityInfo> & RobotHandle::capabilities()
const noexcept
{
  return capabilities_;
}

void RobotHandle::attach(FleetSession & session)
{
  session_ = &session;
}

void RobotHandle::set_capabilities(
  std::vector<easyfleet_mission_manager::CapabilityInfo> capabilities)
{
  capabilities_ = std::move(capabilities);
}

void RobotHandle::note_heartbeat(
  const std::string & capability_type, std::chrono::steady_clock::time_point when)
{
  last_heartbeat_[capability_type] = when;
}

void RobotHandle::check_timeouts()
{
  for (auto & [capability_type, running] : running_) {
    (void)capability_type;
    running->check_timeout();
  }
}

}  // namespace easyfleet
