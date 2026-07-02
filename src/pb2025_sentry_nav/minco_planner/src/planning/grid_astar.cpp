// Copyright 2026

#include "minco_planner/planning/grid_astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace minco_planner
{

namespace
{

struct QueueNode
{
  int x = 0;
  int y = 0;
  double f = 0.0;
  double g = 0.0;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

}  // namespace

GridAstar::GridAstar(GridAstarParams params)
: params_(params)
{
}

void GridAstar::setParams(const GridAstarParams & params)
{
  params_ = params;
}

GridAstarResult GridAstar::plan(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  GridAstarResult result;
  result.path.header = grid.header;

  if (grid.info.width == 0 || grid.info.height == 0 || grid.info.resolution <= 0.0 ||
    grid.data.empty())
  {
    result.reason = "grid is empty";
    return result;
  }

  GridIndex start_idx;
  GridIndex goal_idx;
  if (!worldToGrid(grid, start.pose.position.x, start.pose.position.y, start_idx)) {
    result.reason = "start is outside grid";
    return result;
  }
  if (!worldToGrid(grid, goal.pose.position.x, goal.pose.position.y, goal_idx)) {
    result.reason = "goal is outside grid";
    return result;
  }
  if (!isTraversable(grid, start_idx)) {
    result.reason = "start is occupied";
    return result;
  }
  if (!isTraversable(grid, goal_idx)) {
    result.reason = "goal is occupied";
    return result;
  }

  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<int> parent(cell_count, -1);
  std::vector<uint8_t> closed(cell_count, 0);
  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open;

  const std::size_t start_linear = linearIndex(grid, start_idx);
  const std::size_t goal_linear = linearIndex(grid, goal_idx);
  g_score[start_linear] = 0.0;
  open.push(QueueNode {
      start_idx.x,
      start_idx.y,
      heuristic(start_idx, goal_idx),
      0.0});

  static constexpr int kDx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static constexpr int kDy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  static constexpr int kDx4[4] = {1, 0, -1, 0};
  static constexpr int kDy4[4] = {0, 1, 0, -1};
  const int neighbor_count = params_.allow_diagonal ? 8 : 4;

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();
    GridIndex current_idx {current.x, current.y};
    const std::size_t current_linear = linearIndex(grid, current_idx);
    if (closed[current_linear]) {
      continue;
    }
    closed[current_linear] = 1;
    ++result.expanded_nodes;

    if (current_linear == goal_linear) {
      result.success = true;
      break;
    }

    for (int i = 0; i < neighbor_count; ++i) {
      const int dx = params_.allow_diagonal ? kDx8[i] : kDx4[i];
      const int dy = params_.allow_diagonal ? kDy8[i] : kDy4[i];
      GridIndex next {current.x + dx, current.y + dy};
      if (next.x < 0 || next.y < 0 ||
        next.x >= static_cast<int>(grid.info.width) ||
        next.y >= static_cast<int>(grid.info.height) ||
        !isTraversable(grid, next))
      {
        continue;
      }
      if (params_.allow_diagonal && dx != 0 && dy != 0) {
        // Avoid diagonal corner cutting through two blocked cells. The later
        // footprint checker still verifies body clearance, but the graph search
        // should not create topologically invalid shortcuts.
        if (!isTraversable(grid, GridIndex {current.x + dx, current.y}) ||
          !isTraversable(grid, GridIndex {current.x, current.y + dy}))
        {
          continue;
        }
      }

      const std::size_t next_linear = linearIndex(grid, next);
      if (closed[next_linear]) {
        continue;
      }

      const double step_cost =
        (dx != 0 && dy != 0) ? params_.diagonal_cost : 1.0;
      const double tentative_g = g_score[current_linear] + step_cost;
      if (tentative_g + 1e-9 >= g_score[next_linear]) {
        continue;
      }

      parent[next_linear] = static_cast<int>(current_linear);
      g_score[next_linear] = tentative_g;
      open.push(QueueNode {
          next.x,
          next.y,
          tentative_g + heuristic(next, goal_idx),
          tentative_g});
    }
  }

  if (!result.success) {
    result.reason = "no path";
    return result;
  }

  std::vector<GridIndex> reversed;
  for (int cursor = static_cast<int>(goal_linear); cursor >= 0; cursor = parent[cursor]) {
    const int width = static_cast<int>(grid.info.width);
    reversed.push_back(GridIndex {cursor % width, cursor / width});
    if (static_cast<std::size_t>(cursor) == start_linear) {
      break;
    }
  }
  std::reverse(reversed.begin(), reversed.end());

  result.path.poses.reserve(reversed.size());
  for (const auto & index : reversed) {
    auto pose = gridToPose(grid, index);
    pose.header = result.path.header;
    result.path.poses.push_back(pose);
  }

  for (std::size_t i = 1; i < result.path.poses.size(); ++i) {
    const auto & a = result.path.poses[i - 1].pose.position;
    const auto & b = result.path.poses[i].pose.position;
    result.length += std::hypot(b.x - a.x, b.y - a.y);
  }
  result.reason = "ok";
  return result;
}

bool GridAstar::worldToGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  double wx,
  double wy,
  GridIndex & out) const
{
  const double gx = (wx - grid.info.origin.position.x) / grid.info.resolution;
  const double gy = (wy - grid.info.origin.position.y) / grid.info.resolution;
  if (gx < 0.0 || gy < 0.0 ||
    gx >= static_cast<double>(grid.info.width) ||
    gy >= static_cast<double>(grid.info.height))
  {
    return false;
  }
  out.x = static_cast<int>(std::floor(gx));
  out.y = static_cast<int>(std::floor(gy));
  return true;
}

geometry_msgs::msg::PoseStamped GridAstar::gridToPose(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = grid.header;
  pose.pose.position.x =
    grid.info.origin.position.x + (static_cast<double>(index.x) + 0.5) * grid.info.resolution;
  pose.pose.position.y =
    grid.info.origin.position.y + (static_cast<double>(index.y) + 0.5) * grid.info.resolution;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.w = 1.0;
  return pose;
}

bool GridAstar::isTraversable(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  if (index.x < 0 || index.y < 0 ||
    index.x >= static_cast<int>(grid.info.width) ||
    index.y >= static_cast<int>(grid.info.height))
  {
    return false;
  }

  const int8_t value = grid.data[linearIndex(grid, index)];
  if (value < 0) {
    return !params_.unknown_is_obstacle;
  }
  return value < params_.obstacle_value_threshold;
}

std::size_t GridAstar::linearIndex(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridIndex & index) const
{
  return static_cast<std::size_t>(index.y) * static_cast<std::size_t>(grid.info.width) +
    static_cast<std::size_t>(index.x);
}

double GridAstar::heuristic(const GridIndex & from, const GridIndex & to) const
{
  const double dx = static_cast<double>(from.x - to.x);
  const double dy = static_cast<double>(from.y - to.y);
  return params_.allow_diagonal ? std::hypot(dx, dy) : (std::abs(dx) + std::abs(dy));
}

}  // namespace minco_planner
