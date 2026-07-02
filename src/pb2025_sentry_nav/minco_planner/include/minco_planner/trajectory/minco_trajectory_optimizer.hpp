// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
#define MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_

#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"

namespace minco_planner
{

struct MincoTrajectoryOptimizerParams
{
  double reference_speed = 1.5;
  double min_segment_time = 0.05;
  double sample_spacing = 0.12;
};

class MincoTrajectoryOptimizer
{
public:
  explicit MincoTrajectoryOptimizer(
    MincoTrajectoryOptimizerParams params = MincoTrajectoryOptimizerParams());

  void setParams(const MincoTrajectoryOptimizerParams & params);
  ReferenceTrajectory optimize(const nav_msgs::msg::Path & raw_path) const;

private:
  MincoTrajectoryOptimizerParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_TRAJECTORY_OPTIMIZER_HPP_
