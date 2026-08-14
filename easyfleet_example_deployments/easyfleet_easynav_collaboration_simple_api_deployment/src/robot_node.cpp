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

// The "simple API" mirror of easyfleet_easynav_collaboration_deployment's
// own robot_node.cpp: hosts the exact same fake capabilities, reads the
// exact same "capabilities"/"config_subdir" parameters (e.g.
// config/robot_1/robot_params.yaml), and produces identical resulting
// capability/action/node names -- but driven through
// easyfleet_core::Deployment instead of that package's hand-rolled
// configure/activate/spin/deactivate/cleanup/shutdown dance and hardcoded
// if-chain factory. The two packages are meant to be interchangeable at
// the launch level (see refactor.md for the full design history): point a
// launch file at this package instead of
// easyfleet_easynav_collaboration_deployment and everything else -- Gazebo,
// navigation, RViz, the mission script -- behaves the same.

#include "easyfleet_core/deployment.hpp"
#include "easyfleet_core/init.hpp"

int main(int argc, char ** argv)
{
  easyfleet::init(argc, argv);

  easyfleet::Deployment deployment;
  deployment.add_capabilities_from_parameters(
    "easyfleet_easynav_collaboration_simple_api_deployment");

  deployment.start();
  deployment.run();
  deployment.shutdown();
  return 0;
}
