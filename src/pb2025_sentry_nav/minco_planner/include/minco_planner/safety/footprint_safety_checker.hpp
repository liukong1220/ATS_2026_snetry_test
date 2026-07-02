// Copyright 2026

#ifndef MINCO_PLANNER__FOOTPRINT_SAFETY_CHECKER_HPP_
#define MINCO_PLANNER__FOOTPRINT_SAFETY_CHECKER_HPP_

#include <vector>

#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

struct FootprintSafetyParams
{
  double length = 0.70;
  double width = 0.55;
  double safety_margin = 0.05;
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
};

struct CollisionSample
{
  std::size_t trajectory_index = 0;
  double x = 0.0;
  double y = 0.0;
};

struct FootprintSafetyResult
{
  bool safe = true;
  std::vector<CollisionSample> collisions;
};

class FootprintSafetyChecker
{
public:
  explicit FootprintSafetyChecker(FootprintSafetyParams params = FootprintSafetyParams());

  void setParams(const FootprintSafetyParams & params);
  FootprintSafetyResult check(
    const ReferenceTrajectory & trajectory,
    const nav_msgs::msg::OccupancyGrid & grid) const;

private:
  struct GridIndex
  {
    int x = 0;
    int y = 0;
  };

  bool worldToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    double wx,
    double wy,
    GridIndex & out) const;
  bool isOccupied(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  bool sampleFootprintOccupied(
    const ReferencePoint & point,
    const nav_msgs::msg::OccupancyGrid & grid,
    double & collision_x,
    double & collision_y) const;

  FootprintSafetyParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__FOOTPRINT_SAFETY_CHECKER_HPP_
