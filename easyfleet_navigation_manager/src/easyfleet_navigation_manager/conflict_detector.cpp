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

#include "easyfleet_navigation_manager/conflict_detector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include "geometry_msgs/msg/point.hpp"

namespace easyfleet
{

namespace
{

double distance_xy(
  const geometry_msgs::msg::Point & a,
  const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::hypot(dx, dy);
}

/// Total arc length of the whole path.
double path_length(const nav_msgs::msg::Path & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i) {
    length += distance_xy(path.poses[i - 1].pose.position, path.poses[i].pose.position);
  }
  return length;
}

/// Points from the front of the path up to (and including) the first one
/// at or beyond `lookahead_distance_m` of cumulative arc length -- i.e.
/// the "next lookahead_distance_m of travel", or the whole path if it's
/// shorter than that.
std::vector<geometry_msgs::msg::Point> lookahead_points(
  const nav_msgs::msg::Path & path,
  double lookahead_distance_m)
{
  std::vector<geometry_msgs::msg::Point> points;
  if (path.poses.empty()) {return points;}

  points.push_back(path.poses.front().pose.position);
  double accumulated = 0.0;
  for (std::size_t i = 1; i < path.poses.size() && accumulated < lookahead_distance_m; ++i) {
    accumulated += distance_xy(path.poses[i - 1].pose.position, path.poses[i].pose.position);
    points.push_back(path.poses[i].pose.position);
  }
  return points;
}

/// Minimum distance between any point of `a` and any point of `b`.
/// Point-to-point rather than full segment-to-segment geometry: adequate
/// given planner-published paths are finely, near-uniformly sampled.
double min_distance(
  const std::vector<geometry_msgs::msg::Point> & a,
  const std::vector<geometry_msgs::msg::Point> & b)
{
  double best = std::numeric_limits<double>::max();
  for (const auto & pa : a) {
    for (const auto & pb : b) {
      best = std::min(best, distance_xy(pa, pb));
    }
  }
  return best;
}

}  // namespace

std::set<std::string> compute_robots_to_pause(
  const std::map<std::string, nav_msgs::msg::Path> & robot_paths,
  const ConflictParams & params,
  const std::set<std::string> & already_paused,
  std::vector<ConflictDecision> * decisions)
{
  std::set<std::string> to_pause;

  for (auto it_a = robot_paths.begin(); it_a != robot_paths.end(); ++it_a) {
    if (it_a->second.poses.empty()) {continue;}

    auto it_b = it_a;
    for (++it_b; it_b != robot_paths.end(); ++it_b) {
      if (it_b->second.poses.empty()) {continue;}

      const auto & current_a = it_a->second.poses.front().pose.position;
      const auto & current_b = it_b->second.poses.front().pose.position;
      const double current_distance = distance_xy(current_a, current_b);
      if (current_distance > params.proximity_radius_m) {continue;}

      const auto lookahead_a = lookahead_points(it_a->second, params.lookahead_distance_m);
      const auto lookahead_b = lookahead_points(it_b->second, params.lookahead_distance_m);
      const double lookahead_distance = min_distance(lookahead_a, lookahead_b);
      const bool in_conflict = lookahead_distance <= params.path_conflict_distance_m;

      std::string yielding_robot;
      std::ostringstream reason;

      if (in_conflict) {
        const bool a_paused = already_paused.count(it_a->first) > 0;
        const bool b_paused = already_paused.count(it_b->first) > 0;

        if (a_paused && !b_paused) {
          // it_a is already stationary (cmd_vel zeroed): it cannot itself
          // be a moving threat, so it simply keeps yielding and it_b is
          // never newly paused because of this pair -- regardless of how
          // each robot's remaining path length compares right now.
          yielding_robot = it_a->first;
          reason << it_a->first << " is already paused and keeps yielding; " <<
            it_b->first << " is free to continue";
        } else if (b_paused && !a_paused) {
          yielding_robot = it_b->first;
          reason << it_b->first << " is already paused and keeps yielding; " <<
            it_a->first << " is free to continue";
        } else if (a_paused && b_paused) {
          to_pause.insert(it_a->first);
          to_pause.insert(it_b->first);
          reason << it_a->first << " and " << it_b->first <<
            " are both already paused; no new decision needed";
        } else {
          const double length_a = path_length(it_a->second);
          const double length_b = path_length(it_b->second);
          yielding_robot = (length_a >= length_b) ? it_a->first : it_b->first;
          reason << "paths converge (" << lookahead_distance << "m apart); " <<
            yielding_robot << " has the longer remaining path (" <<
            length_a << "m vs " << length_b << "m) and yields";
        }

        if (!yielding_robot.empty()) {
          to_pause.insert(yielding_robot);
        }
      } else {
        reason << it_a->first << " and " << it_b->first << " are " << current_distance <<
          "m apart (within proximity) but their paths don't converge (closest lookahead " <<
          "distance " << lookahead_distance << "m > threshold): no risk";
      }

      if (decisions != nullptr) {
        ConflictDecision decision;
        decision.robot_a = it_a->first;
        decision.robot_b = it_b->first;
        decision.current_distance_m = current_distance;
        decision.min_lookahead_distance_m = lookahead_distance;
        decision.in_conflict = in_conflict;
        decision.yielding_robot = yielding_robot;
        decision.reason = reason.str();
        decisions->push_back(decision);
      }
    }
  }

  return to_pause;
}

}  // namespace easyfleet
