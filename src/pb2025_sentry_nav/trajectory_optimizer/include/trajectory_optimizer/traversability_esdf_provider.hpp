// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TRAVERSABILITY_ESDF_PROVIDER_HPP_
#define TRAJECTORY_OPTIMIZER__TRAVERSABILITY_ESDF_PROVIDER_HPP_

#include <mutex>
#include <vector>

#include <Eigen/Core>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "trajectory_optimizer/esdf_provider.hpp"

namespace trajectory_optimizer
{

struct RollingWindowBounds
{
  bool valid = false;
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
};

class TraversabilityEsdfProvider : public EsdfProvider
{
public:
  TraversabilityEsdfProvider() = default;

  void configureRollingWindow(bool enabled, double size_x, double size_y);
  void setSlopeGridMaxDegrees(double max_degrees);

  void updateGrid(
    const nav_msgs::msg::OccupancyGrid & traversability_grid,
    int obstacle_value_threshold,
    bool unknown_is_obstacle,
    int lethal_value_threshold = 100,
    const nav_msgs::msg::OccupancyGrid * height_diff_grid = nullptr,
    const nav_msgs::msg::OccupancyGrid * occupancy_ratio_grid = nullptr,
    const nav_msgs::msg::OccupancyGrid * ground_confidence_grid = nullptr,
    const nav_msgs::msg::OccupancyGrid * slope_grid = nullptr);

  bool available() const override;
  double getDistance(double x, double y) const override;
  Eigen::Vector2d getGradient(double x, double y) const override;
  double getSlope(double x, double y) const override;
  bool isInsideLocalWindow(double x, double y) const override;
  bool query(double x, double y, EsdfQueryResult & result) const override;

  RollingWindowBounds getRollingWindowBounds() const;
  double getFootprintClearance(
    const Eigen::Vector2d & position,
    double yaw,
    const std::vector<Eigen::Vector2d> & footprint_samples) const;

private:
  bool worldToGrid(double wx, double wy, double & gx, double & gy) const;
  std::size_t indexOf(unsigned int mx, unsigned int my) const;
  double bilinearDistanceAt(const std::vector<double> & field, double gx, double gy) const;
  Eigen::Vector2d bilinearGradientAt(const std::vector<double> & field, double gx, double gy) const;
  void rebuildSmoothedDistanceField();
  bool extractGridValues(
    const nav_msgs::msg::OccupancyGrid & grid,
    std::vector<double> & values) const;
  bool extractSlopeValues(
    const nav_msgs::msg::OccupancyGrid & grid,
    std::vector<double> & values) const;
  double combineSemanticScore(
    double traversability_score,
    double height_diff_score,
    double occupancy_ratio_score,
    double ground_confidence_score) const;
  bool isInsideRollingWindowUnlocked(double wx, double wy) const;
  void updateRollingWindowBoundsUnlocked();
  void rebuildSignedDistanceField(
    const std::vector<uint8_t> & obstacle_mask,
    const std::vector<uint8_t> & free_mask);

  mutable std::mutex mutex_;
  std::vector<double> distance_field_;
  std::vector<double> distance_to_obstacle_field_;
  std::vector<double> distance_to_free_field_;
  std::vector<double> smoothed_distance_field_;
  std::vector<double> slope_field_;
  unsigned int width_ = 0;
  unsigned int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  bool available_ = false;
  bool rolling_window_enabled_ = true;
  double rolling_window_size_x_ = 0.0;
  double rolling_window_size_y_ = 0.0;
  double slope_grid_max_degrees_ = 45.0;
  RollingWindowBounds rolling_window_bounds_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TRAVERSABILITY_ESDF_PROVIDER_HPP_
