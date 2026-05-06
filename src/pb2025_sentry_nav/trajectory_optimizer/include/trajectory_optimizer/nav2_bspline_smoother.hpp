// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__NAV2_BSPLINE_SMOOTHER_HPP_
#define TRAJECTORY_OPTIMIZER__NAV2_BSPLINE_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_optimizer/bspline_path_optimizer.hpp"

namespace trajectory_optimizer
{

class Nav2BSplineSmoother : public nav2_core::Smoother
{
public:
  Nav2BSplineSmoother() = default;
  ~Nav2BSplineSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  bool smooth(
    nav_msgs::msg::Path & path,
    const rclcpp::Duration & max_time) override;

private:
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("Nav2BSplineSmoother")};
  BSplinePathOptimizer optimizer_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__NAV2_BSPLINE_SMOOTHER_HPP_
