// Copyright 2026

#ifndef MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_
#define MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_

#include <vector>

#include "std_msgs/msg/header.hpp"

namespace minco_planner
{

struct ReferencePoint
{
  double t = 0.0;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  double yaw_rate = 0.0;
  double clearance = 0.0;
  double slope = 0.0;
};

struct ReferenceTrajectory
{
  std_msgs::msg::Header header;
  std::vector<ReferencePoint> points;

  bool empty() const
  {
    return points.empty();
  }

  double totalLength() const
  {
    return points.empty() ? 0.0 : points.back().s;
  }

  double totalTime() const
  {
    return points.empty() ? 0.0 : points.back().t;
  }
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__REFERENCE_TRAJECTORY_HPP_
