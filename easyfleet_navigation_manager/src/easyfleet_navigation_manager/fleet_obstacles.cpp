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

#include "easyfleet_navigation_manager/fleet_obstacles.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace easyfleet
{

namespace
{

/// Longest chain from the base to the map that is followed.
constexpr int kMaxChain = 8;

bool newer(const builtin_interfaces::msg::Time & a, const builtin_interfaces::msg::Time & b)
{
  return a.sec > b.sec || (a.sec == b.sec && a.nanosec > b.nanosec);
}

}  // namespace

RobotPoseTracker::RobotPoseTracker(std::string robot_id, std::string base_frame)
: map_frame_(robot_id + "/map"), base_frame_(robot_id + "/" + base_frame)
{
}

void RobotPoseTracker::update(
  const tf2_msgs::msg::TFMessage & msg, std::chrono::steady_clock::time_point now)
{
  bool changed = false;
  for (const auto & transform : msg.transforms) {
    auto & stored = transforms_[transform.child_frame_id];
    // Ignores an older transform arriving late.
    if (stored.child_frame_id.empty() || !newer(stored.header.stamp, transform.header.stamp)) {
      stored = transform;
      changed = true;
    }
  }
  if (changed) {
    recompute(now);
  }
}

void RobotPoseTracker::recompute(std::chrono::steady_clock::time_point now)
{
  tf2::Transform map_to_base = tf2::Transform::getIdentity();
  builtin_interfaces::msg::Time stamp;
  std::string frame = base_frame_;
  for (int i = 0; i < kMaxChain && frame != map_frame_; ++i) {
    const auto it = transforms_.find(frame);
    if (it == transforms_.end()) {
      return;
    }
    tf2::Transform step;
    tf2::fromMsg(it->second.transform, step);
    map_to_base = step * map_to_base;
    if (newer(it->second.header.stamp, stamp)) {
      stamp = it->second.header.stamp;
    }
    frame = it->second.header.frame_id;
  }
  if (frame != map_frame_) {
    return;
  }
  pose_ = RobotPose{map_to_base.getOrigin().x(), map_to_base.getOrigin().y(), stamp, now};
}

sensor_msgs::msg::PointCloud2 make_fleet_obstacle_cloud(
  const std::string & receiver, const std::map<std::string, RobotPose> & poses,
  const FleetObstacleParams & params)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = receiver + "/map";
  const auto own = poses.find(receiver);
  if (own != poses.end()) {
    cloud.header.stamp = own->second.stamp;
  }

  // A filled disc: concentric rings, about `spacing` apart, plus the center.
  std::vector<std::pair<double, double>> disc{{0.0, 0.0}};
  const double spacing = std::max(params.spacing_m, 0.01);
  const int rings = std::max(1, static_cast<int>(std::ceil(params.radius_m / spacing)));
  for (int ring = 1; ring <= rings; ++ring) {
    const double r = params.radius_m * ring / rings;
    const int n = std::max(6, static_cast<int>(std::ceil(2.0 * M_PI * r / spacing)));
    for (int k = 0; k < n; ++k) {
      const double a = 2.0 * M_PI * k / n;
      disc.emplace_back(r * std::cos(a), r * std::sin(a));
    }
  }

  std::vector<std::pair<double, double>> centers;
  if (own != poses.end()) {
    for (const auto & [robot, pose] : poses) {
      if (robot != receiver) {
        centers.emplace_back(pose.x, pose.y);
      }
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(centers.size() * disc.size());
  cloud.height = 1;
  cloud.width = static_cast<uint32_t>(centers.size() * disc.size());
  if (cloud.width == 0) {
    return cloud;  // the iterators need data
  }
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (const auto & [cx, cy] : centers) {
    for (const auto & [dx, dy] : disc) {
      *x = static_cast<float>(cx + dx);
      *y = static_cast<float>(cy + dy);
      *z = static_cast<float>(params.height_m);
      ++x;
      ++y;
      ++z;
    }
  }
  return cloud;
}

}  // namespace easyfleet
