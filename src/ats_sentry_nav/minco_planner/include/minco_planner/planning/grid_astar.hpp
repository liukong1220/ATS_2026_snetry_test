// Copyright 2026

#ifndef MINCO_PLANNER__GRID_ASTAR_HPP_
#define MINCO_PLANNER__GRID_ASTAR_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace minco_planner
{

struct GridAstarParams
{
  int obstacle_value_threshold = 50;
  bool unknown_is_obstacle = false;
  bool allow_diagonal = true;
  double diagonal_cost = 1.41421356237;
};

struct GridAstarResult
{
  bool success = false;
  std::string reason;
  nav_msgs::msg::Path path;
  double length = 0.0;
  int expanded_nodes = 0;
};

class GridAstar
{
public:
  explicit GridAstar(GridAstarParams params = GridAstarParams());

  void setParams(const GridAstarParams & params);
  GridAstarResult plan(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) const;

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
  geometry_msgs::msg::PoseStamped gridToPose(
    const nav_msgs::msg::OccupancyGrid & grid,
    const GridIndex & index) const;
  bool isTraversable(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  std::size_t linearIndex(const nav_msgs::msg::OccupancyGrid & grid, const GridIndex & index) const;
  double heuristic(const GridIndex & from, const GridIndex & to) const;

  GridAstarParams params_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__GRID_ASTAR_HPP_
