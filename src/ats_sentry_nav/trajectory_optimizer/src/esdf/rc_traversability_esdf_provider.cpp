// Copyright 2026

#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"

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

double decodeScalarGridValue(int8_t value, double max_value)
{
  if (value < 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  // terrain_analysis_ext publishes scalar debug / semantic grids in 0~100.
  // Decode them back into a physical quantity here so all downstream modules
  // consume meaningful units instead of transport-specific encodings.
  return clamp01(static_cast<double>(value) / 100.0) * std::max(max_value, 0.0);
}

}  // namespace

void RcTraversabilityEsdfProvider::configureRollingWindow(bool enabled, double size_x, double size_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  rolling_window_enabled_ = enabled;
  rolling_window_size_x_ = std::max(0.0, size_x);
  rolling_window_size_y_ = std::max(0.0, size_y);
  updateRollingWindowBoundsUnlocked();
}

void RcTraversabilityEsdfProvider::setSlopeGridMaxDegrees(double max_degrees)
{
  std::lock_guard<std::mutex> lock(mutex_);
  slope_grid_max_degrees_ = std::max(1e-3, max_degrees);
}

void RcTraversabilityEsdfProvider::updateGrid(
  const nav_msgs::msg::OccupancyGrid & traversability_grid,
  int obstacle_value_threshold,
  bool unknown_is_obstacle,
  int lethal_value_threshold,
  const nav_msgs::msg::OccupancyGrid * height_diff_grid,
  const nav_msgs::msg::OccupancyGrid * occupancy_ratio_grid,
  const nav_msgs::msg::OccupancyGrid * ground_confidence_grid,
  const nav_msgs::msg::OccupancyGrid * slope_grid)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Treat each incoming traversability grid as a fresh local snapshot.
  // This keeps behavior deterministic and avoids stale semantics surviving across updates.
  available_ = false;
  distance_field_.clear();
  distance_to_obstacle_field_.clear();
  distance_to_free_field_.clear();
  smoothed_distance_field_.clear();
  slope_field_.clear();
  width_ = traversability_grid.info.width;
  height_ = traversability_grid.info.height;
  resolution_ = traversability_grid.info.resolution;
  origin_x_ = traversability_grid.info.origin.position.x;
  origin_y_ = traversability_grid.info.origin.position.y;
  updateRollingWindowBoundsUnlocked();

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
  std::vector<double> slope_values;
  if (height_diff_grid && !extractGridValues(*height_diff_grid, height_values)) {
    height_values.clear();
  }
  if (occupancy_ratio_grid && !extractGridValues(*occupancy_ratio_grid, occupancy_values)) {
    occupancy_values.clear();
  }
  if (ground_confidence_grid && !extractGridValues(*ground_confidence_grid, ground_values)) {
    ground_values.clear();
  }
  if (slope_grid && !extractSlopeValues(*slope_grid, slope_values)) {
    slope_values.clear();
  }
  if (!slope_values.empty()) {
    slope_field_ = slope_values;
  } else {
    // Missing slope data is not fatal for task 1. We keep ESDF available and simply
    // report slope as NaN so task 2 can decide whether to treat that as "unknown" or "flat".
    slope_field_.assign(cell_count, std::numeric_limits<double>::quiet_NaN());
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
        // The traversability_grid is still the primary "can pass / cannot pass" source.
        // Height / occupancy / ground confidence only refine this semantic score so the
        // ESDF obstacle set better matches terrain-analysis intent.
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

bool RcTraversabilityEsdfProvider::available() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return available_ && !distance_field_.empty() && width_ > 1 && height_ > 1;
}

double RcTraversabilityEsdfProvider::getDistance(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  double gx = 0.0;
  double gy = 0.0;
  if (!available_ || !worldToGrid(x, y, gx, gy)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return bilinearDistanceAt(distance_field_, gx, gy);
}

Eigen::Vector2d RcTraversabilityEsdfProvider::getGradient(double x, double y) const
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

double RcTraversabilityEsdfProvider::getSlope(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  double gx = 0.0;
  double gy = 0.0;
  if (!available_ || !worldToGrid(x, y, gx, gy) || slope_field_.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return bilinearDistanceAt(slope_field_, gx, gy);
}

bool RcTraversabilityEsdfProvider::isInsideLocalWindow(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return isInsideRollingWindowUnlocked(x, y);
}

bool RcTraversabilityEsdfProvider::query(double x, double y, EsdfQueryResult & result) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  result = EsdfQueryResult {};
  result.inside_local_window = isInsideRollingWindowUnlocked(x, y);
  if (!available_ || !result.inside_local_window) {
    return false;
  }

  double gx = 0.0;
  double gy = 0.0;
  if (!worldToGrid(x, y, gx, gy)) {
    return false;
  }

  const auto & gradient_field =
    smoothed_distance_field_.empty() ? distance_field_ : smoothed_distance_field_;
  // Keep distance from the raw signed field, but use the smoothed field for gradients.
  // This preserves collision meaning while avoiding noisy finite-difference directions.
  result.distance = bilinearDistanceAt(distance_field_, gx, gy);
  result.gradient = bilinearGradientAt(gradient_field, gx, gy);
  if (!slope_field_.empty()) {
    result.slope = bilinearDistanceAt(slope_field_, gx, gy);
  }
  result.valid = std::isfinite(result.distance);
  return result.valid;
}

RollingWindowBounds RcTraversabilityEsdfProvider::getRollingWindowBounds() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rolling_window_bounds_;
}

double RcTraversabilityEsdfProvider::getFootprintClearance(
  const Eigen::Vector2d & position,
  double yaw,
  const std::vector<Eigen::Vector2d> & footprint_samples) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!available_ || footprint_samples.empty() ||
    !isInsideRollingWindowUnlocked(position.x(), position.y()))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Sample each footprint point after rigidly transforming it by (x, y, yaw).
  // This is intentionally simple for V1: once a collision segment is found later,
  // upper layers can choose whether to only warn, slow down, or locally repair.
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  double min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & sample : footprint_samples) {
    const double wx = position.x() + cos_yaw * sample.x() - sin_yaw * sample.y();
    const double wy = position.y() + sin_yaw * sample.x() + cos_yaw * sample.y();
    double gx = 0.0;
    double gy = 0.0;
    if (!worldToGrid(wx, wy, gx, gy)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double distance = bilinearDistanceAt(distance_field_, gx, gy);
    if (!std::isfinite(distance)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    min_clearance = std::min(min_clearance, distance);
  }

  return std::isfinite(min_clearance) ?
    min_clearance : std::numeric_limits<double>::quiet_NaN();
}

bool RcTraversabilityEsdfProvider::worldToGrid(double wx, double wy, double & gx, double & gy) const
{
  if (resolution_ <= 0.0 || width_ == 0 || height_ == 0) {
    return false;
  }

  // Rolling-window rejection happens before interpolation lookup so callers can
  // distinguish "outside local domain" from "inside domain but close to obstacle".
  if (!isInsideRollingWindowUnlocked(wx, wy)) {
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

std::size_t RcTraversabilityEsdfProvider::indexOf(unsigned int mx, unsigned int my) const
{
  return static_cast<std::size_t>(my) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(mx);
}

bool RcTraversabilityEsdfProvider::extractGridValues(
  const nav_msgs::msg::OccupancyGrid & grid,
  std::vector<double> & values) const
{
  // All semantic side grids must align exactly with the traversability grid.
  // We fail closed here so later debugging can immediately detect mis-wired topics.
  if (
    grid.info.width != width_ || grid.info.height != height_ ||
    std::abs(grid.info.resolution - resolution_) > 1e-6 ||
    std::abs(grid.info.origin.position.x - origin_x_) > 1e-6 ||
    std::abs(grid.info.origin.position.y - origin_y_) > 1e-6)
  {
    return false;
  }

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

bool RcTraversabilityEsdfProvider::extractSlopeValues(
  const nav_msgs::msg::OccupancyGrid & grid,
  std::vector<double> & values) const
{
  if (
    grid.info.width != width_ || grid.info.height != height_ ||
    std::abs(grid.info.resolution - resolution_) > 1e-6 ||
    std::abs(grid.info.origin.position.x - origin_x_) > 1e-6 ||
    std::abs(grid.info.origin.position.y - origin_y_) > 1e-6)
  {
    return false;
  }

  const std::size_t cell_count =
    static_cast<std::size_t>(grid.info.width) * static_cast<std::size_t>(grid.info.height);
  if (grid.data.size() < cell_count || cell_count == 0) {
    return false;
  }

  values.assign(cell_count, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t i = 0; i < cell_count; ++i) {
    // Store slope in degrees rather than normalized 0~1 so the next speed-governor
    // stage can work directly with human-readable thresholds.
    values[i] = decodeScalarGridValue(grid.data[i], slope_grid_max_degrees_);
  }
  return true;
}

double RcTraversabilityEsdfProvider::combineSemanticScore(
  double traversability_score,
  double height_diff_score,
  double occupancy_ratio_score,
  double ground_confidence_score) const
{
  const double safe_height = clamp01(height_diff_score);
  const double occupancy = clamp01(occupancy_ratio_score);
  const double ground = clamp01(ground_confidence_score);
  const double traversability = clamp01(traversability_score);
  // Current weights intentionally favor traversability + height difference.
  // The goal in task 1 is not perfect semantic classification, but to build an
  // obstacle set that is conservative enough for smoothing in narrow passages.
  return clamp01(
    0.45 * traversability +
    0.30 * safe_height +
    0.15 * occupancy +
    0.10 * (1.0 - ground));
}

double RcTraversabilityEsdfProvider::bilinearDistanceAt(
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

Eigen::Vector2d RcTraversabilityEsdfProvider::bilinearGradientAt(
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

void RcTraversabilityEsdfProvider::rebuildSignedDistanceField(
  const std::vector<uint8_t> & obstacle_mask,
  const std::vector<uint8_t> & free_mask)
{
  const std::size_t cell_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  distance_to_obstacle_field_.assign(cell_count, std::numeric_limits<double>::infinity());
  distance_to_free_field_.assign(cell_count, std::numeric_limits<double>::infinity());
  distance_field_.assign(cell_count, std::numeric_limits<double>::quiet_NaN());

  auto propagate = [&](const std::vector<uint8_t> & source_mask, std::vector<double> & field) {
      // Dijkstra-style propagation is slower than a highly optimized EDT, but is easy
      // to maintain and sufficient for the current local grid sizes used in V1 task 1.
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
    // Standard signed-distance construction used by the planner stack:
    // positive in free space, negative in obstacle space, zero near the boundary.
    // The optimizer interprets larger values as safer clearance.
    distance_field_[i] = d_occ - d_free;
  }
}

void RcTraversabilityEsdfProvider::rebuildSmoothedDistanceField()
{
  // Only the gradient field is smoothed. The signed distance itself stays raw so
  // obstacle penetration semantics do not drift due to filtering.
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

bool RcTraversabilityEsdfProvider::isInsideRollingWindowUnlocked(double wx, double wy) const
{
  if (!rolling_window_bounds_.valid) {
    return false;
  }

  return
    wx >= rolling_window_bounds_.min_x && wx <= rolling_window_bounds_.max_x &&
    wy >= rolling_window_bounds_.min_y && wy <= rolling_window_bounds_.max_y;
}

void RcTraversabilityEsdfProvider::updateRollingWindowBoundsUnlocked()
{
  rolling_window_bounds_ = RollingWindowBounds {};
  if (width_ == 0 || height_ == 0 || resolution_ <= 0.0) {
    return;
  }

  // By default the whole received grid is queryable.
  const double grid_min_x = origin_x_;
  const double grid_min_y = origin_y_;
  const double grid_max_x = origin_x_ + static_cast<double>(width_) * resolution_;
  const double grid_max_y = origin_y_ + static_cast<double>(height_) * resolution_;

  rolling_window_bounds_.valid = true;
  rolling_window_bounds_.min_x = grid_min_x;
  rolling_window_bounds_.min_y = grid_min_y;
  rolling_window_bounds_.max_x = grid_max_x;
  rolling_window_bounds_.max_y = grid_max_y;

  if (!rolling_window_enabled_) {
    return;
  }

  if (rolling_window_size_x_ <= 0.0 && rolling_window_size_y_ <= 0.0) {
    // Zero means "use the whole grid even though rolling-window mode is conceptually enabled".
    return;
  }

  // For task 1 we center the explicit local window on the received grid bounds.
  // If later we switch to a robot-centered buffer, only this helper should need to change.
  const double grid_size_x = grid_max_x - grid_min_x;
  const double grid_size_y = grid_max_y - grid_min_y;
  const double effective_size_x =
    rolling_window_size_x_ > 0.0 ? std::min(rolling_window_size_x_, grid_size_x) : grid_size_x;
  const double effective_size_y =
    rolling_window_size_y_ > 0.0 ? std::min(rolling_window_size_y_, grid_size_y) : grid_size_y;
  const double center_x = 0.5 * (grid_min_x + grid_max_x);
  const double center_y = 0.5 * (grid_min_y + grid_max_y);

  rolling_window_bounds_.min_x = center_x - 0.5 * effective_size_x;
  rolling_window_bounds_.max_x = center_x + 0.5 * effective_size_x;
  rolling_window_bounds_.min_y = center_y - 0.5 * effective_size_y;
  rolling_window_bounds_.max_y = center_y + 0.5 * effective_size_y;
}

}  // namespace trajectory_optimizer
