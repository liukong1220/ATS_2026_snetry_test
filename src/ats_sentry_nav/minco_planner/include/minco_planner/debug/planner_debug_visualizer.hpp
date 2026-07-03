// Copyright 2026

#ifndef MINCO_PLANNER__PLANNER_DEBUG_VISUALIZER_HPP_
#define MINCO_PLANNER__PLANNER_DEBUG_VISUALIZER_HPP_

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace minco_planner
{

class PlannerDebugVisualizer
{
public:
  visualization_msgs::msg::MarkerArray buildMarkers(
    const nav_msgs::msg::Path & raw_path,
    const ReferenceTrajectory & trajectory,
    const FootprintSafetyResult & safety) const;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__PLANNER_DEBUG_VISUALIZER_HPP_
