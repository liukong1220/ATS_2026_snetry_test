// Copyright 2026

#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner
{

namespace
{

double poseDistance(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(
    b.pose.position.x - a.pose.position.x,
    b.pose.position.y - a.pose.position.y);
}

}  // namespace

MincoTrajectoryOptimizer::MincoTrajectoryOptimizer(MincoTrajectoryOptimizerParams params)
: params_(params)
{
}

void MincoTrajectoryOptimizer::setParams(const MincoTrajectoryOptimizerParams & params)
{
  params_ = params;
}

ReferenceTrajectory MincoTrajectoryOptimizer::optimize(const nav_msgs::msg::Path & raw_path) const
{
  ReferenceTrajectory trajectory;
  trajectory.header = raw_path.header;
  if (raw_path.poses.empty()) {
    return trajectory;
  }

  const double speed = std::max(0.05, params_.reference_speed);
  const double sample_spacing = std::max(0.02, params_.sample_spacing);
  double accumulated_s = 0.0;
  double accumulated_t = 0.0;

  auto append_point = [&](double x, double y, double s, double t) {
      ReferencePoint point;
      point.x = x;
      point.y = y;
      point.s = s;
      point.t = t;
      point.v = speed;
      trajectory.points.push_back(point);
    };

  append_point(
    raw_path.poses.front().pose.position.x,
    raw_path.poses.front().pose.position.y,
    0.0,
    0.0);

  for (std::size_t i = 1; i < raw_path.poses.size(); ++i) {
    const auto & prev = raw_path.poses[i - 1];
    const auto & next = raw_path.poses[i];
    const double segment_length = poseDistance(prev, next);
    if (segment_length <= 1e-6) {
      continue;
    }

    const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / sample_spacing)));
    for (int step = 1; step <= steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      const double x =
        prev.pose.position.x + ratio * (next.pose.position.x - prev.pose.position.x);
      const double y =
        prev.pose.position.y + ratio * (next.pose.position.y - prev.pose.position.y);
      const double ds = segment_length / static_cast<double>(steps);
      accumulated_s += ds;
      accumulated_t += std::max(params_.min_segment_time, ds / speed);
      append_point(x, y, accumulated_s, accumulated_t);
    }
  }

  return trajectory;
}

}  // namespace minco_planner
