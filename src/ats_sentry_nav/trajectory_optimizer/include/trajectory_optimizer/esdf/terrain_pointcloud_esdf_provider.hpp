// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TERRAIN_POINTCLOUD_ESDF_PROVIDER_HPP_
#define TRAJECTORY_OPTIMIZER__TERRAIN_POINTCLOUD_ESDF_PROVIDER_HPP_

#include <mutex>
#include <vector>

#include <Eigen/Core>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "trajectory_optimizer/esdf/esdf_provider.hpp"

namespace trajectory_optimizer
{

class TerrainPointCloudEsdfProvider : public EsdfProvider
{
public:
  TerrainPointCloudEsdfProvider() = default;

  void updatePointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    double resolution,
    double padding,
    double obstacle_inflation_radius,
    double min_intensity);

  bool available() const override;
  double getDistance(double x, double y) const override;
  Eigen::Vector2d getGradient(double x, double y) const override;

private:
  bool worldToGrid(double wx, double wy, double & gx, double & gy) const;
  std::size_t indexOf(unsigned int mx, unsigned int my) const;
  double bilinearDistanceAt(const std::vector<double> & field, double gx, double gy) const;
  Eigen::Vector2d bilinearGradientAt(const std::vector<double> & field, double gx, double gy) const;
  void rebuildSmoothedDistanceField();

  mutable std::mutex mutex_;
  std::vector<double> distance_field_;
  std::vector<double> smoothed_distance_field_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  bool available_ = false;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TERRAIN_POINTCLOUD_ESDF_PROVIDER_HPP_
