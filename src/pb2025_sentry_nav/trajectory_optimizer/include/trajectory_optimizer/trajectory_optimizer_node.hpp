// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_
#define TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_

#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/trajectory_profile_msg.hpp"
#include "trajectory_optimizer/bspline_path_optimizer.hpp"

namespace trajectory_optimizer
{

class TrajectoryOptimizerNode : public rclcpp::Node
{
public:
  explicit TrajectoryOptimizerNode(const rclcpp::NodeOptions & options);

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  BSplinePathOptimizer optimizer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<sp_msgs::msg::TrajectoryProfileMsg>::SharedPtr profile_pub_;

  std::string input_path_topic_;
  std::string output_path_topic_;
  std::string output_profile_topic_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_
