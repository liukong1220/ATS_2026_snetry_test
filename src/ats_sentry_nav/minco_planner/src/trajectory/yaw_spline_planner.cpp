// Copyright 2026

#include "minco_planner/trajectory/yaw_spline_planner.hpp"

#include <algorithm>
#include <cmath>

namespace minco_planner
{

YawSplinePlanner::YawSplinePlanner(YawSplinePlannerParams params)
: params_(params)
{
}

void YawSplinePlanner::setParams(const YawSplinePlannerParams & params)
{
  params_ = params;
}

void YawSplinePlanner::apply(ReferenceTrajectory & trajectory, double initial_yaw) const
{
  if (trajectory.points.empty()) {
    return;
  }

  double previous_yaw = normalizeAngle(initial_yaw);
  double previous_t = trajectory.points.front().t;
  trajectory.points.front().yaw = previous_yaw;
  trajectory.points.front().yaw_rate = 0.0;

  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & prev = trajectory.points[i - 1];
    auto & current = trajectory.points[i];
    const double tangent_yaw = std::atan2(current.y - prev.y, current.x - prev.x);
    const double dt = std::max(1e-3, current.t - previous_t);
    const double max_delta = std::max(0.0, params_.yaw_rate_limit) * dt;
    const double desired_delta = shortestAngularDistance(previous_yaw, tangent_yaw);
    const double clamped_delta = std::max(-max_delta, std::min(max_delta, desired_delta));

    current.yaw = normalizeAngle(previous_yaw + clamped_delta);
    current.yaw_rate = clamped_delta / dt;
    previous_yaw = current.yaw;
    previous_t = current.t;
  }
}

double YawSplinePlanner::normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double YawSplinePlanner::shortestAngularDistance(double from, double to)
{
  return normalizeAngle(to - from);
}

}  // namespace minco_planner
