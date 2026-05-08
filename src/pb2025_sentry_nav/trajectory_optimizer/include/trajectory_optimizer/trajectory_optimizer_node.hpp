// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_
#define TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_

#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/trajectory_profile_msg.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "trajectory_optimizer/bspline_path_optimizer.hpp"
#include "trajectory_optimizer/fake_costmap_esdf_provider.hpp"

namespace trajectory_optimizer
{

class TrajectoryOptimizerNode : public rclcpp::Node
{
public:
  explicit TrajectoryOptimizerNode(const rclcpp::NodeOptions & options);

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void publishEsdfDebugMarkers(const nav_msgs::msg::Path & path);

  BSplinePathOptimizer optimizer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<sp_msgs::msg::TrajectoryProfileMsg>::SharedPtr profile_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr esdf_marker_pub_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<FakeCostmapEsdfProvider> fake_esdf_provider_;
  OptimizerParams params_;

  std::string input_path_topic_;
  std::string output_path_topic_;
  std::string output_profile_topic_;
  std::string costmap_topic_;
  std::string esdf_debug_topic_{"trajectory_esdf_debug"};
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_
