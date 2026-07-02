// Copyright 2026

#include "minco_planner/safety/local_collision_repair.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace minco_planner
{

LocalCollisionRepair::LocalCollisionRepair(LocalCollisionRepairParams params)
: params_(params)
{
}

void LocalCollisionRepair::setParams(const LocalCollisionRepairParams & params)
{
  params_ = params;
}

bool LocalCollisionRepair::repair(
  ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & collisions,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  if (!params_.enabled || collisions.collisions.empty() || grid.info.resolution <= 0.0) {
    return false;
  }

  bool changed = false;
  for (const auto & collision : collisions.collisions) {
    if (collision.trajectory_index >= trajectory.points.size()) {
      continue;
    }
    auto & point = trajectory.points[collision.trajectory_index];
    double repaired_x = point.x;
    double repaired_y = point.y;
    if (!findNearestFreeCell(grid, point.x, point.y, repaired_x, repaired_y)) {
      continue;
    }
    point.x = repaired_x;
    point.y = repaired_y;
    changed = true;
  }
  return changed;
}

bool LocalCollisionRepair::findNearestFreeCell(
  const nav_msgs::msg::OccupancyGrid & grid,
  double x,
  double y,
  double & repaired_x,
  double & repaired_y) const
{
  const int center_x = static_cast<int>(
    std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
  const int center_y = static_cast<int>(
    std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
  const int radius_cells = std::max(
    1, static_cast<int>(std::ceil(params_.search_radius / grid.info.resolution)));

  double best_distance_sq = std::numeric_limits<double>::infinity();
  bool found = false;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = center_x + dx;
      const int my = center_y + dy;
      if (!isFree(grid, mx, my)) {
        continue;
      }
      const double wx =
        grid.info.origin.position.x + (static_cast<double>(mx) + 0.5) * grid.info.resolution;
      const double wy =
        grid.info.origin.position.y + (static_cast<double>(my) + 0.5) * grid.info.resolution;
      const double distance_sq = (wx - x) * (wx - x) + (wy - y) * (wy - y);
      if (distance_sq >= best_distance_sq) {
        continue;
      }
      best_distance_sq = distance_sq;
      repaired_x = wx;
      repaired_y = wy;
      found = true;
    }
  }
  return found;
}

bool LocalCollisionRepair::isFree(const nav_msgs::msg::OccupancyGrid & grid, int mx, int my) const
{
  if (mx < 0 || my < 0 ||
    mx >= static_cast<int>(grid.info.width) ||
    my >= static_cast<int>(grid.info.height))
  {
    return false;
  }
  const std::size_t linear =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(mx);
  const int8_t value = grid.data[linear];
  if (value < 0) {
    return !params_.unknown_is_obstacle;
  }
  return value < params_.obstacle_value_threshold;
}

}  // namespace minco_planner
