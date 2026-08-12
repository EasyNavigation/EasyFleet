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

#include "easyfleet_easynav_navigation/bt_nodes/finish.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace easyfleet_easynav_navigation
{

namespace
{
constexpr std::chrono::seconds kFinishDuration{2};
}  // namespace

Finish::Finish(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList Finish::providedPorts()
{
  return {};
}

BT::NodeStatus Finish::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  std::cout << "[Finish] Finishing navigation..." << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Finish::onRunning()
{
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed >= kFinishDuration) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void Finish::onHalted()
{
}

}  // namespace easyfleet_easynav_navigation
