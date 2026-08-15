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

#ifndef EASYFLEET_NAVIGATION_MANAGER__CONFLICT_DETECTOR_HPP_
#define EASYFLEET_NAVIGATION_MANAGER__CONFLICT_DETECTOR_HPP_

#include <map>
#include <set>
#include <string>
#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace easyfleet
{

/// @brief Tunable geometry for imminent-conflict detection between two
/// robots' planned paths.
struct ConflictParams
{
  /// @brief How far ahead along each path (arc length, in meters) to look
  /// when checking whether the two robots are about to obstruct each other.
  double lookahead_distance_m {2.0};
  /// @brief How close two robots' lookahead segments must come to each
  /// other, at some sampled pair of points, to count as a conflict.
  double path_conflict_distance_m {0.6};
  /// @brief How close the robots' current positions (path.poses.front())
  /// must be for a conflict to even be considered -- matches the
  /// "están cerca y es evidente que se van a estorbar" two-part
  /// requirement: proximity now, and about to cross paths.
  double proximity_radius_m {3.0};
};

/// @brief Explains one pair's evaluation -- only produced for pairs that
/// are at least within ConflictParams::proximity_radius_m of each other
/// (pairs nowhere near each other aren't worth reporting). Meant to be
/// logged by the caller so an operator can see, live, why a robot was
/// (or wasn't) paused.
struct ConflictDecision
{
  std::string robot_a;
  std::string robot_b;
  /// @brief Distance between the two robots' current positions.
  double current_distance_m {0.0};
  /// @brief Closest distance found between the two lookahead segments.
  double min_lookahead_distance_m {0.0};
  /// @brief Whether this pair is currently considered an imminent conflict.
  bool in_conflict {false};
  /// @brief Which robot yields (stays/becomes paused) because of this
  /// pair. Empty if `in_conflict` is false.
  std::string yielding_robot;
  /// @brief Human-readable explanation of the decision, e.g. "robot_a is
  /// already paused and keeps yielding; robot_b is free to continue", or
  /// "paths converge (0.42m apart); robot_b has the longer remaining
  /// path (4.10m vs 1.00m) and yields".
  std::string reason;
};

/// @brief Decide which robots should currently be paused to avoid an
/// imminent navigation conflict.
///
/// For every pair of robots whose current positions (the first pose of
/// each path) are within `params.proximity_radius_m` of each other, and
/// whose lookahead segments (the first `params.lookahead_distance_m` of
/// arc length from the front of each path) come within
/// `params.path_conflict_distance_m` of each other at some sampled pair
/// of points, one robot yields (is added to the result):
/// - If neither robot in the pair is already in `already_paused`, the one
///   with the *longer* remaining path (farther from its own goal) yields
///   -- the one closer to finishing keeps moving, clearing the conflict
///   zone sooner for both.
/// - If exactly one robot in the pair is already in `already_paused`, it
///   keeps yielding regardless of path length -- a stationary robot
///   (linear velocity zero, since pausing zeroes cmd_vel) cannot itself
///   be a *moving* threat, so the other robot is always free to
///   continue rather than being newly paused because of it. Without
///   this rule, a purely path-length-based comparison could flip which
///   robot yields mid-conflict as the paused one's replanned remaining
///   path changes length while it sits still, needlessly pausing the
///   robot that was already making way.
/// - If both are already in `already_paused`, both remain in the result
///   (no new decision needed).
///
/// Robots with an empty path are skipped entirely (nothing to conflict
/// with; also nothing to pause). The lookahead-segment proximity check
/// samples path poses directly (point-to-point, not point-to-segment):
/// planner-published paths are finely sampled (grid-resolution spacing),
/// so this is an adequate approximation without full segment geometry.
///
/// @param robot_paths Each robot's most recently published planned path,
///   keyed by robot id (e.g. "robot_1").
/// @param params Detection thresholds; see ConflictParams.
/// @param already_paused Robots the caller currently has paused (e.g.
///   because of a still-active conflict with a third robot, or a
///   previous tick's decision on this same pair) -- see the yielding
///   rules above.
/// @param decisions Optional: if non-null, appended with one
///   ConflictDecision per pair that was at least within
///   `params.proximity_radius_m` (whether or not it ended up in
///   conflict), for the caller to log.
/// @return The set of robot ids that should be paused right now. Calling
///   this again with updated paths and diffing against a previous result
///   is how a caller detects when to resume a robot (it drops out of the
///   returned set once the conflict clears).
std::set<std::string> compute_robots_to_pause(
  const std::map<std::string, nav_msgs::msg::Path> & robot_paths,
  const ConflictParams & params = ConflictParams(),
  const std::set<std::string> & already_paused = {},
  std::vector<ConflictDecision> * decisions = nullptr);

}  // namespace easyfleet

#endif  // EASYFLEET_NAVIGATION_MANAGER__CONFLICT_DETECTOR_HPP_
