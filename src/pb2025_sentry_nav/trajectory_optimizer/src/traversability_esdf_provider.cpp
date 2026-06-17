// Copyright 2026

#include "trajectory_optimizer/traversability_esdf_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
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

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

double normalizedSemanticValue(int8_t value)
{
  if (value < 0) {
    return -1.0;
  }
  return clamp01(static_cast<double>(value) / 100.0);
}

}  // namespace

void TraversabilityEsdfProvider::updateGrid(
  const nav_msgs::msg::OccupancyGrid & traversability_grid,
  int obstacle_value_threshold,
  bool unknown_is_obstacle,
  int lethal_value_threshold,
  const nav_msgs::msg::OccupancyGrid * height_diff_grid,
  const nav_msgs::msg::OccupancyGrid * occupancy_ratio_grid,
  const nav_msgs::msg::OccupancyGrid * ground_confidence_grid)
{
  std::lock_guard<std::mutex> lock(mutex_);

  available_ = false;
  distance_field_.clear();
  distance_to_obstacle_field_.clear();
  distance_to_free_field_.clear();
  smoothed_distance_field_.clear();
  width_ = traversability_grid.info.width;
  height_ = traversability_grid.info.height;
  resolution_ = traversability_grid.info.resolution;
  origin_x_ = traversability_grid.info.origin.position.x;
  origin_y_ = traversability_grid.info.origin.position.y;

  if (width_ == 0 || height_ == 0 || resolution_ <= 0.0 || traversability_grid.data.empty()) {
    return;
  }

  const std::size_t cell_count =
    static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  if (traversability_grid.data.size() < cell_count) {
    return;
  }

  std::vector<double> height_values;
  std::vector<double> occupancy_values;
  std::vector<double> ground_values;
  if (height_diff_grid && !extractGridValues(*height_diff_grid, height_values)) {
    height_values.clear();
  }
  if (occupancy_ratio_grid && !extractGridValues(*occupancy_ratio_grid, occupancy_values)) {
    occupancy_values.clear();
  }
  if (ground_confidence_grid && !extractGridValues(*ground_confidence_grid, ground_values)) {
    ground_values.clear();
  }

  std::vector<uint8_t> obstacle_mask(cell_count, 0);
  std::vector<uint8_t> free_mask(cell_count, 0);
  const int safe_threshold = std::max(0, std::min(100, obstacle_value_threshold));
  const int lethal_threshold =
    std::max(safe_threshold, std::min(100, lethal_value_threshold));

  for (unsigned int my = 0; my < height_; ++my) {
    for (unsigned int mx = 0; mx < width_; ++mx) {
      const std::size_t idx = indexOf(mx, my);
      const int8_t value = traversability_grid.data[idx];
      const bool is_unknown = value < 0;
      const bool is_lethal_obstacle =
        value >= lethal_threshold || (unknown_is_obstacle && is_unknown);
      const bool is_risk_obstacle = value >= safe_threshold;

      double semantic_score = normalizedSemanticValue(value);
      if (!height_values.empty()) {
        semantic_score = combineSemanticScore(
          semantic_score,
          height_values[idx],
          occupancy_values.empty() ? semantic_score : occupancy_values[idx],
          ground_values.empty() ? semantic_score : ground_values[idx]);
      }

      const bool is_obstacle =
        is_lethal_obstacle || is_risk_obstacle || semantic_score >= 0.5;
      if (is_obstacle) {
        obstacle_mask[idx] = 1;
      } else {
        free_mask[idx] = 1;
      }
    }
  }

  if (std::none_of(obstacle_mask.begin(), obstacle_mask.end(), [](uint8_t v) { return v != 0; })) {
    return;
  }

  rebuildSignedDistanceField(obstacle_mask, free_mask);
  rebuildSmoothedDistanceField();
  available_ = true;
}

bool TraversabilityEsdfProvider::available() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return available_ && !distance_field_.empty() && width_ > 1 && height_ > 1;
}

double TraversabilityEsdfProvider::getDistance(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  double gx = 0.0;
  double gy = 0.0;
  if (!available_ || !worldToGrid(x, y, gx, gy)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return bilinearDistanceAt(distance_field_, gx, gy);
}

Eigen::Vector2d TraversabilityEsdfProvider::getGradient(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  double gx = 0.0;
  double gy = 0.0;
  if (!available_ || !worldToGrid(x, y, gx, gy)) {
    return Eigen::Vector2d::Zero();
  }
  const auto & field = smoothed_distance_field_.empty() ? distance_field_ : smoothed_distance_field_;
  return bilinearGradientAt(field, gx, gy);
}

bool TraversabilityEsdfProvider::worldToGrid(double wx, double wy, double & gx, double & gy) const
{
  if (resolution_ <= 0.0 || width_ == 0 || height_ == 0) {
    return false;
  }

  gx = ((wx - origin_x_) / resolution_) - 0.5;
  gy = ((wy - origin_y_) / resolution_) - 0.5;
  if (gx < 0.0 || gy < 0.0 || gx > static_cast<double>(width_ - 1) ||
    gy > static_cast<double>(height_ - 1))
  {
    return false;
  }

  gx = std::max(0.0, std::min(gx, static_cast<double>(width_ - 1)));
  gy = std::max(0.0, std::min(gy, static_cast<double>(height_ - 1)));
  return true;
}

std::size_t TraversabilityEsdfProvider::indexOf(unsigned int mx, unsigned int my) const
{
  return static_cast<std::size_t>(my) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(mx);
}

bool TraversabilityEsdfProvider::extractGridValues(
  const nav_msgs::msg::OccupancyGrid & grid,
  std::vector<double> & values) const
{
  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (grid.data.size() < cell_count || cell_count == 0) {
    return false;
  }

  values.assign(cell_count, 0.0);
  for (std::size_t i = 0; i < cell_count; ++i) {
    values[i] = normalizedSemanticValue(grid.data[i]);
  }
  return true;
}

double TraversabilityEsdfProvider::combineSemanticScore(
  double traversability_score,
  double height_diff_score,
  double occupancy_ratio_score,
  double ground_confidence_score) const
{
  const double safe_height = clamp01(height_diff_score);
  const double occupancy = clamp01(occupancy_ratio_score);
  const double ground = clamp01(ground_confidence_score);
  const double traversability = clamp01(traversability_score);
  return clamp01(
    0.45 * traversability +
    0.30 * safe_height +
    0.15 * occupancy +
    0.10 * (1.0 - ground));
}

double TraversabilityEsdfProvider::bilinearDistanceAt(
  const std::vector<double> & field,
  double gx,
  double gy) const
{
  if (field.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
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

Eigen::Vector2d TraversabilityEsdfProvider::bilinearGradientAt(
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

void TraversabilityEsdfProvider::rebuildSignedDistanceField(
  const std::vector<uint8_t> & obstacle_mask,
  const std::vector<uint8_t> & free_mask)
{
  const std::size_t cell_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  distance_to_obstacle_field_.assign(cell_count, std::numeric_limits<double>::infinity());
  distance_to_free_field_.assign(cell_count, std::numeric_limits<double>::infinity());
  distance_field_.assign(cell_count, std::numeric_limits<double>::quiet_NaN());

  auto propagate = [&](const std::vector<uint8_t> & source_mask, std::vector<double> & field) {
      std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare> open;
      bool has_seed = false;
      for (unsigned int my = 0; my < height_; ++my) {
        for (unsigned int mx = 0; mx < width_; ++mx) {
          const std::size_t idx = indexOf(mx, my);
          if (source_mask[idx] == 0) {
            continue;
          }
          field[idx] = 0.0;
          open.push(GridNode {mx, my, 0.0});
          has_seed = true;
        }
      }

      if (!has_seed) {
        return;
      }

      static constexpr int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
      static constexpr int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
      while (!open.empty()) {
        const auto current = open.top();
        open.pop();
        const std::size_t current_idx = indexOf(current.mx, current.my);
        if (current.distance > field[current_idx] + 1e-9) {
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
          const std::size_t neighbor_idx = indexOf(
            static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
          if (candidate_distance + 1e-9 >= field[neighbor_idx]) {
            continue;
          }

          field[neighbor_idx] = candidate_distance;
          open.push(GridNode {
              static_cast<unsigned int>(nx),
              static_cast<unsigned int>(ny),
              candidate_distance});
        }
      }
    };

  propagate(obstacle_mask, distance_to_obstacle_field_);
  propagate(free_mask, distance_to_free_field_);

  for (std::size_t i = 0; i < cell_count; ++i) {
    const double d_occ = distance_to_obstacle_field_[i];
    const double d_free = distance_to_free_field_[i];
    const bool have_occ = std::isfinite(d_occ);
    const bool have_free = std::isfinite(d_free);
    if (!have_occ && !have_free) {
      distance_field_[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    if (!have_free) {
      distance_field_[i] = -d_occ;
      continue;
    }
    if (!have_occ) {
      distance_field_[i] = d_free;
      continue;
    }
    distance_field_[i] = d_free - d_occ;
  }
}

void TraversabilityEsdfProvider::rebuildSmoothedDistanceField()
{
  smoothed_distance_field_ = distance_field_;
  if (distance_field_.empty()) {
    return;
  }

  std::vector<double> scratch = smoothed_distance_field_;
  const int kernel_radius = 1;
  for (unsigned int my = 0; my < height_; ++my) {
    for (unsigned int mx = 0; mx < width_; ++mx) {
      double weighted_sum = 0.0;
      double total_weight = 0.0;
      for (int dy = -kernel_radius; dy <= kernel_radius; ++dy) {
        for (int dx = -kernel_radius; dx <= kernel_radius; ++dx) {
          const int nx = static_cast<int>(mx) + dx;
          const int ny = static_cast<int>(my) + dy;
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) {
            continue;
          }
          const double sample = distance_field_[indexOf(
              static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))];
          if (!std::isfinite(sample)) {
            continue;
          }
          const double weight = (dx == 0 && dy == 0) ? 4.0 : ((dx == 0 || dy == 0) ? 2.0 : 1.0);
          weighted_sum += weight * sample;
          total_weight += weight;
        }
      }
      if (total_weight > 0.0) {
        scratch[indexOf(mx, my)] = weighted_sum / total_weight;
      }
    }
  }
  smoothed_distance_field_.swap(scratch);
}

}  // namespace trajectory_optimizer
