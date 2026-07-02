// Copyright 2026

#include "trajectory_optimizer/esdf/fake_costmap_esdf_provider.hpp"

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
  smoothed_distance_field_.clear();

  if (!costmap_) {
    return;
  }

  width_ = costmap_->getSizeInCellsX();
  height_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  if (width_ == 0 || height_ == 0 || resolution_ <= 0.0) {
    return;
  }

  const std::size_t cell_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  distance_field_.assign(cell_count, std::numeric_limits<double>::infinity());

  std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare> open;
  bool has_obstacle_seed = false;
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
      has_obstacle_seed = true;
    }
  }

  if (!has_obstacle_seed) {
    return;
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

  rebuildSmoothedDistanceField();
  available_ = true;
}

bool FakeCostmapEsdfProvider::available() const
{
  return available_ && costmap_ && !distance_field_.empty();
}

double FakeCostmapEsdfProvider::getDistance(double x, double y) const
{
  double gx = 0.0;
  double gy = 0.0;
  if (!available() || !worldToGrid(x, y, gx, gy)) {
    return -1.0;
  }
  return bilinearDistanceAt(distance_field_, gx, gy);
}

Eigen::Vector2d FakeCostmapEsdfProvider::getGradient(double x, double y) const
{
  double gx = 0.0;
  double gy = 0.0;
  if (!available() || !worldToGrid(x, y, gx, gy)) {
    return Eigen::Vector2d::Zero();
  }
  const auto & field = smoothed_distance_field_.empty() ? distance_field_ : smoothed_distance_field_;
  return bilinearGradientAt(field, gx, gy);
}

bool FakeCostmapEsdfProvider::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  if (!costmap_) {
    return false;
  }
  return costmap_->worldToMap(wx, wy, mx, my);
}

bool FakeCostmapEsdfProvider::worldToGrid(double wx, double wy, double & gx, double & gy) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!worldToMap(wx, wy, mx, my)) {
    return false;
  }

  gx = ((wx - origin_x_) / resolution_) - 0.5;
  gy = ((wy - origin_y_) / resolution_) - 0.5;
  gx = std::max(0.0, std::min(gx, static_cast<double>(width_ - 1)));
  gy = std::max(0.0, std::min(gy, static_cast<double>(height_ - 1)));
  return true;
}

std::size_t FakeCostmapEsdfProvider::indexOf(unsigned int mx, unsigned int my) const
{
  return static_cast<std::size_t>(my) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(mx);
}

double FakeCostmapEsdfProvider::distanceAt(int mx, int my) const
{
  if (distance_field_.empty()) {
    return 0.0;
  }

  const int clamped_x = std::max(0, std::min(mx, static_cast<int>(width_) - 1));
  const int clamped_y = std::max(0, std::min(my, static_cast<int>(height_) - 1));
  return distance_field_[indexOf(static_cast<unsigned int>(clamped_x), static_cast<unsigned int>(clamped_y))];
}

double FakeCostmapEsdfProvider::bilinearDistanceAt(
  const std::vector<double> & field,
  double gx,
  double gy) const
{
  if (field.empty()) {
    return -1.0;
  }

  const int x0 = static_cast<int>(std::floor(gx));
  const int y0 = static_cast<int>(std::floor(gy));
  const int x1 = std::min(x0 + 1, static_cast<int>(width_) - 1);
  const int y1 = std::min(y0 + 1, static_cast<int>(height_) - 1);
  const double tx = gx - static_cast<double>(x0);
  const double ty = gy - static_cast<double>(y0);

  const auto sample = [&](int x, int y) {
      const int clamped_x = std::max(0, std::min(x, static_cast<int>(width_) - 1));
      const int clamped_y = std::max(0, std::min(y, static_cast<int>(height_) - 1));
      return field[indexOf(static_cast<unsigned int>(clamped_x), static_cast<unsigned int>(clamped_y))];
    };

  const double d00 = sample(x0, y0);
  const double d10 = sample(x1, y0);
  const double d01 = sample(x0, y1);
  const double d11 = sample(x1, y1);
  return
    (1.0 - tx) * (1.0 - ty) * d00 +
    tx * (1.0 - ty) * d10 +
    (1.0 - tx) * ty * d01 +
    tx * ty * d11;
}

Eigen::Vector2d FakeCostmapEsdfProvider::bilinearGradientAt(
  const std::vector<double> & field,
  double gx,
  double gy) const
{
  if (field.empty()) {
    return Eigen::Vector2d::Zero();
  }

  const int x0 = static_cast<int>(std::floor(gx));
  const int y0 = static_cast<int>(std::floor(gy));
  const int x1 = std::min(x0 + 1, static_cast<int>(width_) - 1);
  const int y1 = std::min(y0 + 1, static_cast<int>(height_) - 1);
  const double tx = gx - static_cast<double>(x0);
  const double ty = gy - static_cast<double>(y0);

  const auto sample = [&](int x, int y) {
      const int clamped_x = std::max(0, std::min(x, static_cast<int>(width_) - 1));
      const int clamped_y = std::max(0, std::min(y, static_cast<int>(height_) - 1));
      return field[indexOf(static_cast<unsigned int>(clamped_x), static_cast<unsigned int>(clamped_y))];
    };

  const double d00 = sample(x0, y0);
  const double d10 = sample(x1, y0);
  const double d01 = sample(x0, y1);
  const double d11 = sample(x1, y1);
  const double denom = std::max(resolution_, 1e-6);

  const double grad_x =
    ((1.0 - ty) * (d10 - d00) + ty * (d11 - d01)) / denom;
  const double grad_y =
    ((1.0 - tx) * (d01 - d00) + tx * (d11 - d10)) / denom;
  return Eigen::Vector2d {grad_x, grad_y};
}

void FakeCostmapEsdfProvider::rebuildSmoothedDistanceField()
{
  smoothed_distance_field_ = distance_field_;
  if (distance_field_.empty()) {
    return;
  }

  static constexpr double kKernel[3] = {1.0, 2.0, 1.0};
  for (unsigned int my = 0; my < height_; ++my) {
    for (unsigned int mx = 0; mx < width_; ++mx) {
      double weighted_sum = 0.0;
      double weight_total = 0.0;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const int sx = std::max(0, std::min(static_cast<int>(mx) + dx, static_cast<int>(width_) - 1));
          const int sy = std::max(0, std::min(static_cast<int>(my) + dy, static_cast<int>(height_) - 1));
          const double weight = kKernel[dx + 1] * kKernel[dy + 1];
          weighted_sum += weight * distance_field_[indexOf(
            static_cast<unsigned int>(sx),
            static_cast<unsigned int>(sy))];
          weight_total += weight;
        }
      }
      smoothed_distance_field_[indexOf(mx, my)] =
        (weight_total > 0.0) ? (weighted_sum / weight_total) : distance_field_[indexOf(mx, my)];
    }
  }
}

}  // namespace trajectory_optimizer
