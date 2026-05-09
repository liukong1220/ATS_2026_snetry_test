// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__NAV2_BSPLINE_SMOOTHER_HPP_
#define TRAJECTORY_OPTIMIZER__NAV2_BSPLINE_SMOOTHER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_core/smoother.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/trajectory_profile_msg.hpp"
#include "trajectory_optimizer/bspline_path_optimizer.hpp"
#include "trajectory_optimizer/fake_costmap_esdf_provider.hpp"

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
  void enforceCostmapClearance(
    nav_msgs::msg::Path & smoothed_path,
    const nav_msgs::msg::Path & reference_path) const;
  void updatePathOrientations(nav_msgs::msg::Path & path) const;
  geometry_msgs::msg::PoseStamped projectTowardReference(
    const geometry_msgs::msg::PoseStamped & current_pose,
    const geometry_msgs::msg::PoseStamped & reference_pose,
    double step_ratio) const;
  double estimatePoseYaw(
    const nav_msgs::msg::Path & path,
    size_t index,
    const geometry_msgs::msg::PoseStamped * override_pose = nullptr) const;
  bool getRobotFootprint(nav2_costmap_2d::Footprint & footprint) const;
  double sampleFootprintCost(
    nav2_costmap_2d::Costmap2D & costmap,
    const nav_msgs::msg::Path & path,
    size_t index,
    const geometry_msgs::msg::PoseStamped * override_pose = nullptr) const;
  void collectCollidingIndices(
    nav2_costmap_2d::Costmap2D & costmap,
    const nav_msgs::msg::Path & path,
    std::vector<size_t> & indices) const;
  bool pathHasBlockingCollision(
    nav2_costmap_2d::Costmap2D & costmap,
    const nav_msgs::msg::Path & path) const;
  bool locallyDegradeCollidingSegments(
    nav2_costmap_2d::Costmap2D & costmap,
    nav_msgs::msg::Path & path,
    const nav_msgs::msg::Path & reference_path) const;
  bool samplePathCost(
    const nav2_costmap_2d::Costmap2D & costmap,
    const geometry_msgs::msg::PoseStamped & pose,
    unsigned char & cost) const;

  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("Nav2BSplineSmoother")};
  rclcpp::Clock::SharedPtr clock_;
  BSplinePathOptimizer optimizer_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub_;
  std::shared_ptr<FakeCostmapEsdfProvider> fake_esdf_provider_;
  rclcpp_lifecycle::LifecyclePublisher<sp_msgs::msg::TrajectoryProfileMsg>::SharedPtr profile_pub_;
  std::string profile_topic_{"trajectory_profile"};
  unsigned char max_path_cost_{96};
  unsigned char footprint_collision_cost_threshold_{253};
  int pullback_samples_{6};
  int collision_skip_initial_points_{4};
  double collision_skip_initial_distance_{0.25};
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__NAV2_BSPLINE_SMOOTHER_HPP_
