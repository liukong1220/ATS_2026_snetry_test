// Copyright 2026

#ifndef MINCO_PLANNER__LOCAL_COLLISION_REPAIR_HPP_
#define MINCO_PLANNER__LOCAL_COLLISION_REPAIR_HPP_

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace minco_planner
{

struct LocalCollisionRepairParams
{
  bool enabled = true;
  int max_iterations = 2;
  double search_radius = 0.35;
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
};

class LocalCollisionRepair
{
public:
  explicit LocalCollisionRepair(LocalCollisionRepairParams params = LocalCollisionRepairParams());

  void setParams(const LocalCollisionRepairParams & params);
  bool repair(
    ReferenceTrajectory & trajectory,
    const FootprintSafetyResult & collisions,
    const nav_msgs::msg::OccupancyGrid & grid) const;

private:
  bool findNearestFreeCell(
    const nav_msgs::msg::OccupancyGrid & grid,
    double x,
    double y,
    double & repaired_x,
    double & repaired_y) const;
  bool isFree(const nav_msgs::msg::OccupancyGrid & grid, int mx, int my) const;

  LocalCollisionRepairParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_COLLISION_REPAIR_HPP_
