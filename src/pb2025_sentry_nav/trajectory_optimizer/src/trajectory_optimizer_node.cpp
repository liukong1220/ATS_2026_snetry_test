// Copyright 2026

#include "trajectory_optimizer/trajectory_optimizer_node.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace trajectory_optimizer
{

namespace
{

sp_msgs::msg::TrajectoryProfileMsg toProfileMsg(
  const std_msgs::msg::Header & header,
  const std::string & source,
  const TrajectoryProfile2D & profile)
{
  sp_msgs::msg::TrajectoryProfileMsg msg;
  msg.header = header;
  msg.source = source;
  msg.total_length = profile.total_length;
  msg.total_time = profile.total_time;
  msg.curvature_penalty = profile.curvature_penalty;
  msg.velocity_smoothness_cost = profile.velocity_smoothness_cost;
  msg.obstacle_cost = profile.obstacle_cost;
  msg.total_cost = profile.total_cost;
  msg.max_abs_curvature = profile.max_abs_curvature;
  msg.points.reserve(profile.samples.size());

  for (const auto & sample : profile.samples) {
    sp_msgs::msg::TrajectoryProfilePoint point;
    point.s = sample.s;
    point.t = sample.t;
    point.point.x = sample.point.x;
    point.point.y = sample.point.y;
    point.point.z = 0.0;
    point.first_derivative.x = sample.first_derivative.x;
    point.first_derivative.y = sample.first_derivative.y;
    point.first_derivative.z = 0.0;
    point.second_derivative.x = sample.second_derivative.x;
    point.second_derivative.y = sample.second_derivative.y;
    point.second_derivative.z = 0.0;
    point.curvature = sample.curvature;
    point.speed_limit = sample.speed_limit;
    point.speed = sample.speed;
    point.acceleration = sample.acceleration;
    msg.points.push_back(point);
  }

  return msg;
}

}  // namespace

TrajectoryOptimizerNode::TrajectoryOptimizerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_optimizer", options)
{
  OptimizerParams params;

  declare_parameter<std::string>("input_path_topic", "plan");
  declare_parameter<std::string>("output_path_topic", "smoothed_path");
  declare_parameter<std::string>("output_profile_topic", "trajectory_profile_visual");
  declare_parameter<double>("control_point_spacing", params.control_point_spacing);
  declare_parameter<double>("output_path_spacing", params.output_path_spacing);
  declare_parameter<double>("min_input_point_spacing", params.min_input_point_spacing);
  declare_parameter<double>("max_lateral_deviation", params.max_lateral_deviation);
  declare_parameter<int>("min_control_points", params.min_control_points);
  declare_parameter<double>("curvature_limit", params.curvature_limit);
  declare_parameter<double>("curvature_weight", params.curvature_weight);
  declare_parameter<int>(
    "curvature_refinement_iterations", params.curvature_refinement_iterations);
  declare_parameter<double>("curvature_refinement_gain", params.curvature_refinement_gain);
  declare_parameter<double>("global_speed_limit", params.global_speed_limit);
  declare_parameter<double>("lateral_accel_limit", params.lateral_accel_limit);
  declare_parameter<double>("longitudinal_accel_limit", params.longitudinal_accel_limit);
  declare_parameter<double>("velocity_smoothing_gain", params.velocity_smoothing_gain);
  declare_parameter<double>("derivative_step", params.derivative_step);

  get_parameter("input_path_topic", input_path_topic_);
  get_parameter("output_path_topic", output_path_topic_);
  get_parameter("output_profile_topic", output_profile_topic_);
  get_parameter("control_point_spacing", params.control_point_spacing);
  get_parameter("output_path_spacing", params.output_path_spacing);
  get_parameter("min_input_point_spacing", params.min_input_point_spacing);
  get_parameter("max_lateral_deviation", params.max_lateral_deviation);
  get_parameter("min_control_points", params.min_control_points);
  get_parameter("curvature_limit", params.curvature_limit);
  get_parameter("curvature_weight", params.curvature_weight);
  get_parameter("curvature_refinement_iterations", params.curvature_refinement_iterations);
  get_parameter("curvature_refinement_gain", params.curvature_refinement_gain);
  get_parameter("global_speed_limit", params.global_speed_limit);
  get_parameter("lateral_accel_limit", params.lateral_accel_limit);
  get_parameter("longitudinal_accel_limit", params.longitudinal_accel_limit);
  get_parameter("velocity_smoothing_gain", params.velocity_smoothing_gain);
  get_parameter("derivative_step", params.derivative_step);
  optimizer_.setParams(params);

  smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);
  profile_pub_ =
    create_publisher<sp_msgs::msg::TrajectoryProfileMsg>(output_profile_topic_, 10);
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
  const auto result = optimizer_.optimizeDetailed(*msg);
  smoothed_path_pub_->publish(result.path);
  profile_pub_->publish(toProfileMsg(msg->header, "trajectory_optimizer_node", result.profile));
}

}  // namespace trajectory_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimizer::TrajectoryOptimizerNode)
