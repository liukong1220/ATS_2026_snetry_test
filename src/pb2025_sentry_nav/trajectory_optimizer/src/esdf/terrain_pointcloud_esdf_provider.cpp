// Copyright 2026

#include "trajectory_optimizer/esdf/terrain_pointcloud_esdf_provider.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>

#include "sensor_msgs/point_field_conversion.hpp"

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

bool pointIsFinite(float x, float y)
{
  return std::isfinite(x) && std::isfinite(y);
}

int findFieldIndex(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
{
  for (std::size_t i = 0; i < cloud.fields.size(); ++i) {
    if (cloud.fields[i].name == field_name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace

void TerrainPointCloudEsdfProvider::updatePointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  double resolution,
  double padding,
  double obstacle_inflation_radius,
  double min_intensity)
{
  std::lock_guard<std::mutex> lock(mutex_);

  available_ = false;
  distance_field_.clear();
  smoothed_distance_field_.clear();
  width_ = 0;
  height_ = 0;
  resolution_ = 0.0;
  origin_x_ = 0.0;
  origin_y_ = 0.0;

  if (cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0) {
    return;
  }

  const double safe_resolution = std::max(0.02, resolution);
  const double safe_padding = std::max(0.10, padding);
  const double inflation_radius = std::max(0.0, obstacle_inflation_radius);

  const int x_offset = findFieldIndex(cloud, "x");
  const int y_offset = findFieldIndex(cloud, "y");
  const int intensity_offset = findFieldIndex(cloud, "intensity");
  if (x_offset < 0 || y_offset < 0) {
    return;
  }
  const int x_field_offset = static_cast<int>(cloud.fields[static_cast<std::size_t>(x_offset)].offset);
  const int y_field_offset = static_cast<int>(cloud.fields[static_cast<std::size_t>(y_offset)].offset);
  const uint8_t x_datatype = cloud.fields[static_cast<std::size_t>(x_offset)].datatype;
  const uint8_t y_datatype = cloud.fields[static_cast<std::size_t>(y_offset)].datatype;
  int intensity_field_offset = -1;
  uint8_t intensity_datatype = sensor_msgs::msg::PointField::FLOAT32;
  if (intensity_offset >= 0) {
    intensity_field_offset =
      static_cast<int>(cloud.fields[static_cast<std::size_t>(intensity_offset)].offset);
    intensity_datatype =
      cloud.fields[static_cast<std::size_t>(intensity_offset)].datatype;
  }

  float min_x = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
  bool has_points = false;

  const std::size_t point_count = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);
  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t offset = i * cloud.point_step;
    const float x = sensor_msgs::readPointCloud2BufferValue<float>(
      &cloud.data[offset + static_cast<std::size_t>(x_field_offset)], x_datatype);
    const float y = sensor_msgs::readPointCloud2BufferValue<float>(
      &cloud.data[offset + static_cast<std::size_t>(y_field_offset)], y_datatype);
    if (!pointIsFinite(x, y)) {
      continue;
    }

    if (intensity_field_offset >= 0) {
      const float intensity = sensor_msgs::readPointCloud2BufferValue<float>(
        &cloud.data[offset + static_cast<std::size_t>(intensity_field_offset)], intensity_datatype);
      if (std::isfinite(intensity) && intensity < static_cast<float>(min_intensity)) {
        continue;
      }
    }

    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
    has_points = true;
  }

  if (!has_points) {
    return;
  }

  origin_x_ = static_cast<double>(min_x) - safe_padding;
  origin_y_ = static_cast<double>(min_y) - safe_padding;
  const double map_width = std::max(safe_resolution, static_cast<double>(max_x - min_x) + 2.0 * safe_padding);
  const double map_height = std::max(safe_resolution, static_cast<double>(max_y - min_y) + 2.0 * safe_padding);
  width_ = static_cast<unsigned int>(std::ceil(map_width / safe_resolution));
  height_ = static_cast<unsigned int>(std::ceil(map_height / safe_resolution));
  resolution_ = safe_resolution;

  if (width_ < 2 || height_ < 2) {
    return;
  }

  const std::size_t cell_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  distance_field_.assign(cell_count, std::numeric_limits<double>::infinity());
  std::vector<uint8_t> occupancy(cell_count, 0);

  const int inflation_cells = static_cast<int>(std::ceil(inflation_radius / safe_resolution));
  for (std::size_t i = 0; i < point_count; ++i) {
    const std::size_t offset = i * cloud.point_step;
    const float x = sensor_msgs::readPointCloud2BufferValue<float>(
      &cloud.data[offset + static_cast<std::size_t>(x_field_offset)], x_datatype);
    const float y = sensor_msgs::readPointCloud2BufferValue<float>(
      &cloud.data[offset + static_cast<std::size_t>(y_field_offset)], y_datatype);
    if (!pointIsFinite(x, y)) {
      continue;
    }

    if (intensity_field_offset >= 0) {
      const float intensity = sensor_msgs::readPointCloud2BufferValue<float>(
        &cloud.data[offset + static_cast<std::size_t>(intensity_field_offset)], intensity_datatype);
      if (std::isfinite(intensity) && intensity < static_cast<float>(min_intensity)) {
        continue;
      }
    }

    const int mx = static_cast<int>(std::floor((static_cast<double>(x) - origin_x_) / safe_resolution));
    const int my = static_cast<int>(std::floor((static_cast<double>(y) - origin_y_) / safe_resolution));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(width_) || my >= static_cast<int>(height_)) {
      continue;
    }

    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        const int nx = mx + dx;
        const int ny = my + dy;
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) || ny >= static_cast<int>(height_)) {
          continue;
        }
        if (inflation_cells > 0) {
          const double dist = std::sqrt(static_cast<double>(dx * dx + dy * dy)) * safe_resolution;
          if (dist > inflation_radius + 1e-9) {
            continue;
          }
        }
        occupancy[indexOf(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))] = 1;
      }
    }
  }

  std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare> open;
  bool has_seed = false;
  for (unsigned int my = 0; my < height_; ++my) {
    for (unsigned int mx = 0; mx < width_; ++mx) {
      if (occupancy[indexOf(mx, my)] == 0) {
        continue;
      }
      distance_field_[indexOf(mx, my)] = 0.0;
      open.push(GridNode {mx, my, 0.0});
      has_seed = true;
    }
  }

  if (!has_seed) {
    distance_field_.clear();
    return;
  }

  static constexpr int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  static constexpr int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();

    const std::size_t current_idx = indexOf(current.mx, current.my);
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
        (kDx[dir] == 0 || kDy[dir] == 0) ? safe_resolution : safe_resolution * std::sqrt(2.0);
      const double candidate_distance = current.distance + step;
      const std::size_t neighbor_idx = indexOf(
        static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
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

bool TerrainPointCloudEsdfProvider::available() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return available_ && !distance_field_.empty() && width_ > 1 && height_ > 1;
}

double TerrainPointCloudEsdfProvider::getDistance(double x, double y) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  double gx = 0.0;
  double gy = 0.0;
  if (!available_ || !worldToGrid(x, y, gx, gy)) {
    return -1.0;
  }
  return bilinearDistanceAt(distance_field_, gx, gy);
}

Eigen::Vector2d TerrainPointCloudEsdfProvider::getGradient(double x, double y) const
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

bool TerrainPointCloudEsdfProvider::worldToGrid(double wx, double wy, double & gx, double & gy) const
{
  if (resolution_ <= 0.0 || width_ == 0 || height_ == 0) {
    return false;
  }

  gx = ((wx - origin_x_) / resolution_) - 0.5;
  gy = ((wy - origin_y_) / resolution_) - 0.5;
  if (gx < 0.0 || gy < 0.0 || gx > static_cast<double>(width_ - 1) || gy > static_cast<double>(height_ - 1)) {
    return false;
  }
  gx = std::max(0.0, std::min(gx, static_cast<double>(width_ - 1)));
  gy = std::max(0.0, std::min(gy, static_cast<double>(height_ - 1)));
  return true;
}

std::size_t TerrainPointCloudEsdfProvider::indexOf(unsigned int mx, unsigned int my) const
{
  return static_cast<std::size_t>(my) * static_cast<std::size_t>(width_) +
    static_cast<std::size_t>(mx);
}

double TerrainPointCloudEsdfProvider::bilinearDistanceAt(
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

Eigen::Vector2d TerrainPointCloudEsdfProvider::bilinearGradientAt(
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

void TerrainPointCloudEsdfProvider::rebuildSmoothedDistanceField()
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
