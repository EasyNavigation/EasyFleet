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

#include "easyfleet_easynav_deployment/bt_nodes/start_off.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace easyfleet_easynav_deployment
{

namespace
{
constexpr std::chrono::seconds kStartOffDuration{2};
}  // namespace

StartOff::StartOff(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList StartOff::providedPorts()
{
  return {};
}

BT::NodeStatus StartOff::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  std::cout << "[StartOff] Starting navigation..." << std::endl;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StartOff::onRunning()
{
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed >= kStartOffDuration) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void StartOff::onHalted()
{
}

}  // namespace easyfleet_easynav_deployment

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<easyfleet_easynav_deployment::StartOff>("StartOff");
}
