// Copyright 2026

#include "trajectory_optimizer/nav2_bspline_smoother.hpp"

#include <algorithm>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

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

void Nav2BSplineSmoother::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
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
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".curvature_limit",
    rclcpp::ParameterValue(params.curvature_limit));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".curvature_weight",
    rclcpp::ParameterValue(params.curvature_weight));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".curvature_refinement_iterations",
    rclcpp::ParameterValue(params.curvature_refinement_iterations));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".curvature_refinement_gain",
    rclcpp::ParameterValue(params.curvature_refinement_gain));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".global_speed_limit",
    rclcpp::ParameterValue(params.global_speed_limit));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".lateral_accel_limit",
    rclcpp::ParameterValue(params.lateral_accel_limit));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".longitudinal_accel_limit",
    rclcpp::ParameterValue(params.longitudinal_accel_limit));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".velocity_smoothing_gain",
    rclcpp::ParameterValue(params.velocity_smoothing_gain));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".derivative_step",
    rclcpp::ParameterValue(params.derivative_step));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".profile_topic",
    rclcpp::ParameterValue(profile_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".max_path_cost",
    rclcpp::ParameterValue(static_cast<int>(max_path_cost_)));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".pullback_samples",
    rclcpp::ParameterValue(pullback_samples_));

  node->get_parameter(plugin_name_ + ".control_point_spacing", params.control_point_spacing);
  node->get_parameter(plugin_name_ + ".output_path_spacing", params.output_path_spacing);
  node->get_parameter(plugin_name_ + ".min_input_point_spacing", params.min_input_point_spacing);
  node->get_parameter(plugin_name_ + ".max_lateral_deviation", params.max_lateral_deviation);
  node->get_parameter(plugin_name_ + ".min_control_points", params.min_control_points);
  node->get_parameter(plugin_name_ + ".curvature_limit", params.curvature_limit);
  node->get_parameter(plugin_name_ + ".curvature_weight", params.curvature_weight);
  node->get_parameter(
    plugin_name_ + ".curvature_refinement_iterations",
    params.curvature_refinement_iterations);
  node->get_parameter(
    plugin_name_ + ".curvature_refinement_gain",
    params.curvature_refinement_gain);
  node->get_parameter(plugin_name_ + ".global_speed_limit", params.global_speed_limit);
  node->get_parameter(plugin_name_ + ".lateral_accel_limit", params.lateral_accel_limit);
  node->get_parameter(
    plugin_name_ + ".longitudinal_accel_limit",
    params.longitudinal_accel_limit);
  node->get_parameter(
    plugin_name_ + ".velocity_smoothing_gain",
    params.velocity_smoothing_gain);
  node->get_parameter(plugin_name_ + ".derivative_step", params.derivative_step);
  node->get_parameter(plugin_name_ + ".profile_topic", profile_topic_);
  int configured_max_cost = static_cast<int>(max_path_cost_);
  node->get_parameter(plugin_name_ + ".max_path_cost", configured_max_cost);
  node->get_parameter(plugin_name_ + ".pullback_samples", pullback_samples_);
  max_path_cost_ = static_cast<unsigned char>(std::max(0, configured_max_cost));

  optimizer_.setParams(params);
  costmap_sub_ = costmap_sub;
  logger_ = node->get_logger();
  profile_pub_ =
    node->create_publisher<sp_msgs::msg::TrajectoryProfileMsg>(profile_topic_, 10);
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
  const nav_msgs::msg::Path reference_path = path;
  if (costmap_sub_) {
    optimizer_.setObstacleCostmap(costmap_sub_->getCostmap());
  } else {
    optimizer_.clearObstacleCostmap();
  }
  auto result = optimizer_.optimizeDetailed(path);
  path = result.path;
  enforceCostmapClearance(path, reference_path);
  if (profile_pub_) {
    profile_pub_->publish(
      toProfileMsg(path.header, "nav2_bspline_smoother", result.profile));
  }
  return true;
}

void Nav2BSplineSmoother::enforceCostmapClearance(
  nav_msgs::msg::Path & smoothed_path,
  const nav_msgs::msg::Path & reference_path) const
{
  if (!costmap_sub_ || smoothed_path.poses.size() < 3 || reference_path.poses.empty()) {
    return;
  }

  auto costmap = costmap_sub_->getCostmap();
  if (!costmap) {
    RCLCPP_WARN(logger_, "No costmap available for bspline smoother clearance enforcement.");
    return;
  }

  const size_t last_reference_idx = reference_path.poses.size() - 1;
  for (size_t i = 1; i + 1 < smoothed_path.poses.size(); ++i) {
    unsigned char cost = nav2_costmap_2d::NO_INFORMATION;
    if (!samplePathCost(*costmap, smoothed_path.poses[i], cost)) {
      continue;
    }
    if (cost <= max_path_cost_) {
      continue;
    }

    const double ratio =
      static_cast<double>(i) / static_cast<double>(smoothed_path.poses.size() - 1);
    const size_t ref_idx = std::min(
      last_reference_idx,
      static_cast<size_t>(std::round(ratio * static_cast<double>(last_reference_idx))));
    const auto original_pose = reference_path.poses[ref_idx];
    const auto current_pose = smoothed_path.poses[i];

    bool repaired = false;
    for (int step = 1; step <= pullback_samples_; ++step) {
      const double alpha = static_cast<double>(step) / static_cast<double>(pullback_samples_);
      auto candidate = current_pose;
      candidate.pose.position.x =
        current_pose.pose.position.x +
        (original_pose.pose.position.x - current_pose.pose.position.x) * alpha;
      candidate.pose.position.y =
        current_pose.pose.position.y +
        (original_pose.pose.position.y - current_pose.pose.position.y) * alpha;

      unsigned char candidate_cost = nav2_costmap_2d::NO_INFORMATION;
      if (!samplePathCost(*costmap, candidate, candidate_cost)) {
        continue;
      }

      if (candidate_cost <= max_path_cost_) {
        smoothed_path.poses[i] = candidate;
        repaired = true;
        break;
      }
    }

    if (!repaired) {
      smoothed_path.poses[i] = original_pose;
    }
  }
}

bool Nav2BSplineSmoother::samplePathCost(
  const nav2_costmap_2d::Costmap2D & costmap,
  const geometry_msgs::msg::PoseStamped & pose,
  unsigned char & cost) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;
  }
  cost = costmap.getCost(mx, my);
  return true;
}

}  // namespace trajectory_optimizer

PLUGINLIB_EXPORT_CLASS(trajectory_optimizer::Nav2BSplineSmoother, nav2_core::Smoother)
