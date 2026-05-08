// Copyright 2026

#include "trajectory_optimizer/fake_costmap_esdf_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace trajectory_optimizer
{

namespace
{

struct GridNode
{
  unsigned int mx = 0;
  unsigned int my = 0;
  double distance = 0.0;
};

struct GridNodeCompare
{
  bool operator()(const GridNode & lhs, const GridNode & rhs) const
  {
    return lhs.distance > rhs.distance;
  }
};

}  // namespace

void FakeCostmapEsdfProvider::updateCostmap(
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap,
  unsigned char obstacle_cost_threshold,
  bool unknown_is_obstacle)
{
  costmap_ = costmap;
  available_ = false;
  distance_field_.clear();

  if (!costmap_) {
    return;
  }

  width_ = costmap_->getSizeInCellsX();
  height_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  if (width_ == 0 || height_ == 0 || resolution_ <= 0.0) {
    return;
  }

  const std::size_t cell_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  distance_field_.assign(cell_count, std::numeric_limits<double>::infinity());

  std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare> open;
  for (unsigned int my = 0; my < height_; ++my) {
    for (unsigned int mx = 0; mx < width_; ++mx) {
      const unsigned char cost = costmap_->getCost(mx, my);
      const bool is_unknown = cost == nav2_costmap_2d::NO_INFORMATION;
      const bool is_obstacle =
        cost >= obstacle_cost_threshold ||
        cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        (unknown_is_obstacle && is_unknown);
      if (!is_obstacle) {
        continue;
      }

      const auto idx = indexOf(mx, my);
      distance_field_[idx] = 0.0;
      open.push(GridNode {mx, my, 0.0});
    }
  }

  static constexpr int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static constexpr int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    const auto current_idx = indexOf(current.mx, current.my);
    if (current.distance > distance_field_[current_idx] + 1e-9) {
      continue;
    }

    for (int dir = 0; dir < 8; ++dir) {
      const int nx = static_cast<int>(current.mx) + kDx[dir];
      const int ny = static_cast<int>(current.my) + kDy[dir];
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) {
        continue;
      }

      const double step =
        (kDx[dir] == 0 || kDy[dir] == 0) ? resolution_ : resolution_ * std::sqrt(2.0);
      const double candidate_distance = current.distance + step;
      const auto neighbor_idx = indexOf(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      if (candidate_distance + 1e-9 >= distance_field_[neighbor_idx]) {
        continue;
      }

      distance_field_[neighbor_idx] = candidate_distance;
      open.push(GridNode {
          static_cast<unsigned int>(nx),
          static_cast<unsigned int>(ny),
          candidate_distance});
    }
  }

  available_ = true;
}

bool FakeCostmapEsdfProvider::available() const
{
  return available_ && costmap_ && !distance_field_.empty();
}

double FakeCostmapEsdfProvider::getDistance(double x, double y) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!available() || !worldToMap(x, y, mx, my)) {
    return -1.0;
  }
  return distance_field_[indexOf(mx, my)];
}

Eigen::Vector2d FakeCostmapEsdfProvider::getGradient(double x, double y) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!available() || !worldToMap(x, y, mx, my)) {
    return Eigen::Vector2d::Zero();
  }

  const int ix = static_cast<int>(mx);
  const int iy = static_cast<int>(my);
  const double left = distanceAt(ix - 1, iy);
  const double right = distanceAt(ix + 1, iy);
  const double down = distanceAt(ix, iy - 1);
  const double up = distanceAt(ix, iy + 1);
  const double denom = std::max(resolution_, 1e-6);
  return Eigen::Vector2d {
    (right - left) / (2.0 * denom),
    (up - down) / (2.0 * denom)};
}

bool FakeCostmapEsdfProvider::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!costmap_) {
    return false;
  }
  return costmap_->worldToMap(wx, wy, mx, my);
}

std::size_t FakeCostmapEsdfProvider::indexOf(unsigned int mx, unsigned int my) const
{
  return static_cast<std::size_t>(my) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(mx);
}

double FakeCostmapEsdfProvider::distanceAt(int mx, int my) const
{
  if (mx < 0 || my < 0 || mx >= static_cast<int>(width_) || my >= static_cast<int>(height_)) {
    return 0.0;
  }
  return distance_field_[indexOf(static_cast<unsigned int>(mx), static_cast<unsigned int>(my))];
}

}  // namespace trajectory_optimizer
