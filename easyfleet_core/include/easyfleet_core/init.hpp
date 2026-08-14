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

#ifndef EASYFLEET_CORE__INIT_HPP_
#define EASYFLEET_CORE__INIT_HPP_

namespace easyfleet
{

/// @brief Every EasyFleet program -- a `Deployment` hosting capabilities, or a
/// mission script driving a `SimpleController` -- starts with this instead
/// of a bare `rclcpp::init(argc, argv)`.
///
/// It's the same call, with one deliberate difference:
/// `SignalHandlerOptions::None` instead of rclcpp's own default SIGINT
/// handling, which is what lets `Deployment::run()` (and
/// `easyfleet_core::spin_until_shutdown()`, which it's built on) shut
/// lifecycle nodes down cleanly on Ctrl-C -- deactivate/cleanup/shutdown
/// each capability -- instead of having the ROS context torn out from under
/// them before they get the chance. See `spin_utils.hpp` for the full
/// explanation of why that matters.
///
/// @param argc Argument count, as received by `main()`.
/// @param argv Argument vector, as received by `main()`.
void init(int argc, char ** argv);

}  // namespace easyfleet

#endif  // EASYFLEET_CORE__INIT_HPP_
