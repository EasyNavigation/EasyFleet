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

#ifndef EASYFLEET_MISSION_MANAGER__MISSION_HELPERS_HPP_
#define EASYFLEET_MISSION_MANAGER__MISSION_HELPERS_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "easyfleet_interfaces/action/manipulation.hpp"
#include "easyfleet_interfaces/action/navigation.hpp"
#include "easyfleet_interfaces/action/perception.hpp"
#include "rclcpp/rclcpp.hpp"

#include "easyfleet_mission_manager/ansi.hpp"
#include "easyfleet_mission_manager/capability_info.hpp"
#include "easyfleet_mission_manager/output.hpp"
#include "easyfleet_mission_manager/throttle.hpp"

// Shared by every mission script (main_alone.cpp, main_collaboration.cpp,
// ...): the easyfleet_interfaces action types the mock capabilities speak,
// how to build a demo goal and a feedback printer for each of them, and
// small terminal/discovery helpers. Kept here instead of duplicated
// per-mission so all scripts stay in sync as capabilities evolve.
namespace easyfleet_mission_manager
{

using Navigation = easyfleet_interfaces::action::Navigation;
using Manipulation = easyfleet_interfaces::action::Manipulation;
using Perception = easyfleet_interfaces::action::Perception;

/// How long a mission script lets a capability run before stopping it, if
/// it hasn't finished on its own by then.
constexpr std::chrono::seconds kRunTimeout(10);

/// Starts `executor.spin()` on a background thread and blocks until it has
/// actually begun spinning before returning it. `Executor::cancel()` only
/// reliably interrupts a `spin()` that has already started.
inline std::thread spin_in_background(rclcpp::Executor & executor)
{
  std::thread thread([&executor] {executor.spin();});
  while (!executor.is_spinning()) {
    std::this_thread::yield();
  }
  return thread;
}

/// Prints a bold, boxed header marking a new phase of a mission script.
inline void print_section(const std::string & title)
{
  std::ostringstream out;
  out << "\n" << ansi::bold << ansi::blue << "== " << title << " ==" << ansi::reset;
  safe_print(out.str());
}

/// Prints a dim, indented line explaining what a phase is about to do (or
/// just did), so the mission's progress is legible as it happens.
inline void print_step(const std::string & text)
{
  safe_print(std::string("  ") + ansi::dim + text + ansi::reset);
}

/// Finds `robot`'s active capability of the given short type (e.g.
/// "navigation"). `action_name` is what must actually be dialed to reach
/// it (e.g. "/robot_1/navigation").
/// @return The matching entry, or `std::nullopt` if none was found -- a
///   nullable *reference* into `capabilities`, without a raw pointer.
inline std::optional<std::reference_wrapper<const CapabilityInfo>> find_robot_capability(
  const std::vector<CapabilityInfo> & capabilities,
  const std::string & robot, const std::string & capability)
{
  auto it = std::find_if(
    capabilities.begin(), capabilities.end(),
    [&](const CapabilityInfo & info) {
      return info.robot == robot && info.capability == capability && info.active;
    });
  if (it == capabilities.end()) {
    return std::nullopt;
  }
  return std::cref(*it);
}

inline Navigation::Goal make_navigation_goal()
{
  Navigation::Goal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;
  return goal;
}

/// @param label Printed on every feedback line, e.g. "/robot_1/navigation":
///   needed to tell apart interleaved feedback from several robots running
///   the same capability type in parallel.
inline std::function<void(const Navigation::Feedback & )> make_navigation_feedback_printer(
  const std::string & label)
{
  Throttle throttle(std::chrono::milliseconds(500));
  return [throttle, label](const Navigation::Feedback & feedback) {
           if (!throttle.ready()) {
             return;
           }
           std::ostringstream out;
           out << "  " << ansi::dim << "[" << label << "] " << ansi::reset
               << "distance_remaining=" << feedback.distance_remaining << "m, "
               << "elapsed=" << feedback.navigation_time.sec << "s, "
               << "recoveries=" << feedback.number_of_recoveries;
           safe_print(out.str());
         };
}

inline Manipulation::Goal make_manipulation_goal()
{
  Manipulation::Goal goal;
  goal.mode = Manipulation::Goal::MODE_JOINT_TARGET;
  goal.joint_target.name = {"joint1"};
  goal.joint_target.position = {1.0};
  return goal;
}

inline std::function<void(const Manipulation::Feedback & )> make_manipulation_feedback_printer(
  const std::string & label)
{
  Throttle throttle(std::chrono::milliseconds(500));
  return [throttle, label](const Manipulation::Feedback & feedback) {
           if (!throttle.ready()) {
             return;
           }
           std::ostringstream out;
           out << "  " << ansi::dim << "[" << label << "] " << ansi::reset
               << "state=" << feedback.state;
           safe_print(out.str());
         };
}

inline Perception::Goal make_perception_goal()
{
  Perception::Goal goal;
  goal.object_classes.push_back("gato");
  return goal;
}

inline std::function<void(const Perception::Feedback & )> make_perception_feedback_printer(
  const std::string & label)
{
  Throttle throttle(std::chrono::milliseconds(500));
  return [throttle, label](const Perception::Feedback & feedback) {
           if (!throttle.ready()) {
             return;
           }
           std::ostringstream out;
           out << "  " << ansi::dim << "[" << label << "] " << ansi::reset
               << feedback.detections_3d.detections.size() << " cat(s) detected (3D + 2D)";
           safe_print(out.str());
         };
}

}  // namespace easyfleet_mission_manager

#endif  // EASYFLEET_MISSION_MANAGER__MISSION_HELPERS_HPP_
