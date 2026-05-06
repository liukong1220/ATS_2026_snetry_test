// Copyright 2026

#include "trajectory_optimizer/nav2_bspline_smoother.hpp"

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace trajectory_optimizer
{

void Nav2BSplineSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock parent node for Nav2BSplineSmoother");
  }

  plugin_name_ = name;

  OptimizerParams params;
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".control_point_spacing",
    rclcpp::ParameterValue(params.control_point_spacing));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".output_path_spacing",
    rclcpp::ParameterValue(params.output_path_spacing));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".min_input_point_spacing",
    rclcpp::ParameterValue(params.min_input_point_spacing));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".max_lateral_deviation",
    rclcpp::ParameterValue(params.max_lateral_deviation));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".min_control_points",
    rclcpp::ParameterValue(params.min_control_points));

  node->get_parameter(plugin_name_ + ".control_point_spacing", params.control_point_spacing);
  node->get_parameter(plugin_name_ + ".output_path_spacing", params.output_path_spacing);
  node->get_parameter(plugin_name_ + ".min_input_point_spacing", params.min_input_point_spacing);
  node->get_parameter(plugin_name_ + ".max_lateral_deviation", params.max_lateral_deviation);
  node->get_parameter(plugin_name_ + ".min_control_points", params.min_control_points);

  optimizer_.setParams(params);
  logger_ = node->get_logger();
  RCLCPP_INFO(logger_, "Configured Nav2BSplineSmoother plugin: %s", plugin_name_.c_str());
}

void Nav2BSplineSmoother::cleanup()
{
}

void Nav2BSplineSmoother::activate()
{
}

void Nav2BSplineSmoother::deactivate()
{
}

bool Nav2BSplineSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration &)
{
  path = optimizer_.optimize(path);
  return true;
}

}  // namespace trajectory_optimizer

PLUGINLIB_EXPORT_CLASS(trajectory_optimizer::Nav2BSplineSmoother, nav2_core::Smoother)
