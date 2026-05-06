// Copyright 2026

#include "trajectory_optimizer/trajectory_optimizer_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"

namespace trajectory_optimizer
{

TrajectoryOptimizerNode::TrajectoryOptimizerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_optimizer", options)
{
  OptimizerParams params;

  declare_parameter<std::string>("input_path_topic", "plan");
  declare_parameter<std::string>("output_path_topic", "smoothed_path");
  declare_parameter<double>("control_point_spacing", params.control_point_spacing);
  declare_parameter<double>("output_path_spacing", params.output_path_spacing);
  declare_parameter<double>("min_input_point_spacing", params.min_input_point_spacing);
  declare_parameter<double>("max_lateral_deviation", params.max_lateral_deviation);
  declare_parameter<int>("min_control_points", params.min_control_points);

  get_parameter("input_path_topic", input_path_topic_);
  get_parameter("output_path_topic", output_path_topic_);
  get_parameter("control_point_spacing", params.control_point_spacing);
  get_parameter("output_path_spacing", params.output_path_spacing);
  get_parameter("min_input_point_spacing", params.min_input_point_spacing);
  get_parameter("max_lateral_deviation", params.max_lateral_deviation);
  get_parameter("min_control_points", params.min_control_points);
  optimizer_.setParams(params);

  smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    input_path_topic_, 10,
    std::bind(&TrajectoryOptimizerNode::pathCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Trajectory optimizer active: %s -> %s",
    input_path_topic_.c_str(), output_path_topic_.c_str());
}

void TrajectoryOptimizerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  smoothed_path_pub_->publish(optimizer_.optimize(*msg));
}

}  // namespace trajectory_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimizer::TrajectoryOptimizerNode)
