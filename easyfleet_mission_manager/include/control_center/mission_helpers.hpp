// Copyright 2026 Intelligent Robotics Lab
//
// This file is part of the projects Arquimea-URJC and AURORAS
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

#ifndef CONTROL_CENTER__MISSION_HELPERS_HPP_
#define CONTROL_CENTER__MISSION_HELPERS_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "moveit_msgs/action/execute_trajectory.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "perception_interfaces/action/perception.hpp"
#include "rclcpp/rclcpp.hpp"

#include "control_center/ansi.hpp"
#include "control_center/capability_info.hpp"
#include "control_center/output.hpp"
#include "control_center/throttle.hpp"

// Shared by every mission script (main_alone.cpp, main_collaboration.cpp,
// ...): the concrete action types the mock capabilities speak, how to
// build a demo goal and a feedback printer for each of them, and small
// terminal/discovery helpers. Kept here instead of duplicated per-mission
// so all scripts stay in sync as capabilities evolve.
namespace control_center
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;
using Perception = perception_interfaces::action::Perception;

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
inline const CapabilityInfo * find_robot_capability(
  const std::vector<CapabilityInfo> & capabilities,
  const std::string & robot, const std::string & capability)
{
  auto it = std::find_if(
    capabilities.begin(), capabilities.end(),
    [&](const CapabilityInfo & info) {
      return info.robot == robot && info.capability == capability && info.active;
    });
  return it != capabilities.end() ? &(*it) : nullptr;
}

inline NavigateToPose::Goal make_navigation_goal()
{
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = 2.0;
  goal.pose.pose.position.y = 1.0;
  goal.pose.pose.orientation.w = 1.0;
  return goal;
}

/// @param label Printed on every feedback line, e.g. "/robot_1/navigation":
///   needed to tell apart interleaved feedback from several robots running
///   the same capability type in parallel.
inline std::function<void(const NavigateToPose::Feedback & )> make_navigation_feedback_printer(
  const std::string & label)
{
  Throttle throttle(std::chrono::milliseconds(500));
  return [throttle, label](const NavigateToPose::Feedback & feedback) {
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

inline ExecuteTrajectory::Goal make_manipulation_goal()
{
  ExecuteTrajectory::Goal goal;
  goal.trajectory.joint_trajectory.joint_names = {"joint1"};

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {1.0};
  point.time_from_start.sec = 1;
  goal.trajectory.joint_trajectory.points.push_back(point);

  return goal;
}

inline std::function<void(const ExecuteTrajectory::Feedback & )> make_manipulation_feedback_printer(
  const std::string & label)
{
  Throttle throttle(std::chrono::milliseconds(500));
  return [throttle, label](const ExecuteTrajectory::Feedback & feedback) {
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
  goal.objects.push_back("gato");
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

}  // namespace control_center

#endif  // CONTROL_CENTER__MISSION_HELPERS_HPP_
