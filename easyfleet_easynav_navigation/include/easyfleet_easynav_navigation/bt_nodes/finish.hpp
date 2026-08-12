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

#ifndef EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__FINISH_HPP_
#define EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__FINISH_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace easyfleet_easynav_navigation
{

/// Bookend BT node run after Navigate: takes 2 seconds and only prints a
/// finishing message. No ports.
class Finish : public BT::StatefulActionNode
{
public:
  Finish(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace easyfleet_easynav_navigation

#endif  // EASYFLEET_EASYNAV_NAVIGATION__BT_NODES__FINISH_HPP_
