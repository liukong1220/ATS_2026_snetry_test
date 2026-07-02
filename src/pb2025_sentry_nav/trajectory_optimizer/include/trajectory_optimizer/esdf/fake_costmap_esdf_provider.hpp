// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__FAKE_COSTMAP_ESDF_PROVIDER_HPP_
#define TRAJECTORY_OPTIMIZER__FAKE_COSTMAP_ESDF_PROVIDER_HPP_

#include <memory>
#include <vector>

#include <Eigen/Core>
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "trajectory_optimizer/esdf/esdf_provider.hpp"

namespace trajectory_optimizer
{

class FakeCostmapEsdfProvider : public EsdfProvider
{
public:
  FakeCostmapEsdfProvider() = default;

  void updateCostmap(
    const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap,
    unsigned char obstacle_cost_threshold,
    bool unknown_is_obstacle);

  bool available() const override;
  double getDistance(double x, double y) const override;
  Eigen::Vector2d getGradient(double x, double y) const override;

private:
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  bool worldToGrid(double wx, double wy, double & gx, double & gy) const;
  std::size_t indexOf(unsigned int mx, unsigned int my) const;
  double distanceAt(int mx, int my) const;
  double bilinearDistanceAt(const std::vector<double> & field, double gx, double gy) const;
  Eigen::Vector2d bilinearGradientAt(const std::vector<double> & field, double gx, double gy) const;
  void rebuildSmoothedDistanceField();

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
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

#endif  // TRAJECTORY_OPTIMIZER__FAKE_COSTMAP_ESDF_PROVIDER_HPP_
