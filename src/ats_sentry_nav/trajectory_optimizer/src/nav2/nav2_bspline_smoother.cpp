// Copyright 2026

#include "trajectory_optimizer/nav2/nav2_bspline_smoother.hpp"

#include <array>
#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace trajectory_optimizer
{

namespace
{

constexpr double kHalfPi = 1.5707963267948966;
constexpr double kPi = 3.1415926535897932;

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
  std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub)
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
    node.get(), plugin_name_ + ".use_continuous_optimization",
    rclcpp::ParameterValue(params.use_continuous_optimization));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".continuous_max_iterations",
    rclcpp::ParameterValue(params.continuous_max_iterations));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".continuous_lbfgs_memory",
    rclcpp::ParameterValue(params.continuous_lbfgs_memory));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".continuous_gradient_tolerance",
    rclcpp::ParameterValue(params.continuous_gradient_tolerance));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".continuous_initial_step",
    rclcpp::ParameterValue(params.continuous_initial_step));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".smoothness_weight",
    rclcpp::ParameterValue(params.smoothness_weight));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".fitness_weight",
    rclcpp::ParameterValue(params.fitness_weight));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".endpoint_tangent_weight",
    rclcpp::ParameterValue(params.endpoint_tangent_weight));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".corridor_weight",
    rclcpp::ParameterValue(params.corridor_weight));
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
    node.get(), plugin_name_ + ".use_slope_speed_limits",
    rclcpp::ParameterValue(params.use_slope_speed_limits));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_speed_boost_start_deg",
    rclcpp::ParameterValue(params.slope_speed_boost_start_deg));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_speed_obstacle_deg",
    rclcpp::ParameterValue(params.slope_speed_obstacle_deg));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_speed_limit_full_deg",
    rclcpp::ParameterValue(params.slope_speed_limit_full_deg));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_speed_max_scale",
    rclcpp::ParameterValue(params.slope_speed_max_scale));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_speed_min_scale",
    rclcpp::ParameterValue(params.slope_speed_min_scale));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".use_slope_accel_limits",
    rclcpp::ParameterValue(params.use_slope_accel_limits));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_accel_boost_start_deg",
    rclcpp::ParameterValue(params.slope_accel_boost_start_deg));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_accel_obstacle_deg",
    rclcpp::ParameterValue(params.slope_accel_obstacle_deg));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_accel_limit_full_deg",
    rclcpp::ParameterValue(params.slope_accel_limit_full_deg));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_accel_max_scale",
    rclcpp::ParameterValue(params.slope_accel_max_scale));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".slope_accel_min_scale",
    rclcpp::ParameterValue(params.slope_accel_min_scale));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".velocity_smoothing_gain",
    rclcpp::ParameterValue(params.velocity_smoothing_gain));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".derivative_step",
    rclcpp::ParameterValue(params.derivative_step));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_safe_cost",
    rclcpp::ParameterValue(static_cast<int>(params.obstacle_safe_cost)));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_weight",
    rclcpp::ParameterValue(params.obstacle_weight));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_refinement_iterations",
    rclcpp::ParameterValue(params.obstacle_refinement_iterations));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_refinement_gain",
    rclcpp::ParameterValue(params.obstacle_refinement_gain));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".use_esdf_obstacle_cost",
    rclcpp::ParameterValue(params.use_esdf_obstacle_cost));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_safe_distance",
    rclcpp::ParameterValue(params.obstacle_safe_distance));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_speed_reduction_distance",
    rclcpp::ParameterValue(params.obstacle_speed_reduction_distance));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_speed_min_distance",
    rclcpp::ParameterValue(params.obstacle_speed_min_distance));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".obstacle_speed_min_scale",
    rclcpp::ParameterValue(params.obstacle_speed_min_scale));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".profile_topic",
    rclcpp::ParameterValue(profile_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".max_path_cost",
    rclcpp::ParameterValue(static_cast<int>(max_path_cost_)));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".footprint_collision_cost_threshold",
    rclcpp::ParameterValue(static_cast<int>(footprint_collision_cost_threshold_)));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".pullback_samples",
    rclcpp::ParameterValue(pullback_samples_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".collision_skip_initial_points",
    rclcpp::ParameterValue(collision_skip_initial_points_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".collision_skip_initial_distance",
    rclcpp::ParameterValue(collision_skip_initial_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".esdf_source",
    rclcpp::ParameterValue(esdf_source_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".terrain_pointcloud_topic",
    rclcpp::ParameterValue(terrain_pointcloud_topic_));
  // Keep the smoother's ESDF input parameters aligned with the standalone
  // trajectory_optimizer node so debugging and BT mainline behavior stay comparable.
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_grid_topic",
    rclcpp::ParameterValue(traversability_grid_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_height_diff_topic",
    rclcpp::ParameterValue(traversability_height_diff_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_occupancy_ratio_topic",
    rclcpp::ParameterValue(traversability_occupancy_ratio_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_ground_confidence_topic",
    rclcpp::ParameterValue(traversability_ground_confidence_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_slope_topic",
    rclcpp::ParameterValue(traversability_slope_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".terrain_esdf_resolution",
    rclcpp::ParameterValue(terrain_esdf_resolution_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".terrain_esdf_padding",
    rclcpp::ParameterValue(terrain_esdf_padding_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".terrain_esdf_inflation_radius",
    rclcpp::ParameterValue(terrain_esdf_inflation_radius_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".terrain_esdf_min_intensity",
    rclcpp::ParameterValue(terrain_esdf_min_intensity_));
  // Same RC-ESDF-lite configuration as the standalone node:
  // this avoids the common handover bug where RViz visualization and Nav2 mainline
  // appear to use "the same ESDF source" but are actually configured differently.
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".rc_esdf_rolling_window_enabled",
    rclcpp::ParameterValue(rc_esdf_rolling_window_enabled_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".rc_esdf_query_window_size_x",
    rclcpp::ParameterValue(rc_esdf_query_window_size_x_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".rc_esdf_query_window_size_y",
    rclcpp::ParameterValue(rc_esdf_query_window_size_y_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_slope_max_degrees",
    rclcpp::ParameterValue(traversability_slope_max_degrees_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_obstacle_value_threshold",
    rclcpp::ParameterValue(traversability_obstacle_value_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_lethal_value_threshold",
    rclcpp::ParameterValue(traversability_lethal_value_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node.get(), plugin_name_ + ".traversability_unknown_is_obstacle",
    rclcpp::ParameterValue(traversability_unknown_is_obstacle_));

  node->get_parameter(plugin_name_ + ".control_point_spacing", params.control_point_spacing);
  node->get_parameter(plugin_name_ + ".output_path_spacing", params.output_path_spacing);
  node->get_parameter(plugin_name_ + ".min_input_point_spacing", params.min_input_point_spacing);
  node->get_parameter(plugin_name_ + ".max_lateral_deviation", params.max_lateral_deviation);
  node->get_parameter(plugin_name_ + ".min_control_points", params.min_control_points);
  node->get_parameter(
    plugin_name_ + ".use_continuous_optimization", params.use_continuous_optimization);
  node->get_parameter(
    plugin_name_ + ".continuous_max_iterations", params.continuous_max_iterations);
  node->get_parameter(
    plugin_name_ + ".continuous_lbfgs_memory", params.continuous_lbfgs_memory);
  node->get_parameter(
    plugin_name_ + ".continuous_gradient_tolerance", params.continuous_gradient_tolerance);
  node->get_parameter(
    plugin_name_ + ".continuous_initial_step", params.continuous_initial_step);
  node->get_parameter(plugin_name_ + ".smoothness_weight", params.smoothness_weight);
  node->get_parameter(plugin_name_ + ".fitness_weight", params.fitness_weight);
  node->get_parameter(
    plugin_name_ + ".endpoint_tangent_weight", params.endpoint_tangent_weight);
  node->get_parameter(plugin_name_ + ".corridor_weight", params.corridor_weight);
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
    plugin_name_ + ".use_slope_speed_limits",
    params.use_slope_speed_limits);
  node->get_parameter(
    plugin_name_ + ".slope_speed_boost_start_deg",
    params.slope_speed_boost_start_deg);
  node->get_parameter(
    plugin_name_ + ".slope_speed_obstacle_deg",
    params.slope_speed_obstacle_deg);
  node->get_parameter(
    plugin_name_ + ".slope_speed_limit_full_deg",
    params.slope_speed_limit_full_deg);
  node->get_parameter(
    plugin_name_ + ".slope_speed_max_scale",
    params.slope_speed_max_scale);
  node->get_parameter(
    plugin_name_ + ".slope_speed_min_scale",
    params.slope_speed_min_scale);
  node->get_parameter(
    plugin_name_ + ".use_slope_accel_limits",
    params.use_slope_accel_limits);
  node->get_parameter(
    plugin_name_ + ".slope_accel_boost_start_deg",
    params.slope_accel_boost_start_deg);
  node->get_parameter(
    plugin_name_ + ".slope_accel_obstacle_deg",
    params.slope_accel_obstacle_deg);
  node->get_parameter(
    plugin_name_ + ".slope_accel_limit_full_deg",
    params.slope_accel_limit_full_deg);
  node->get_parameter(
    plugin_name_ + ".slope_accel_max_scale",
    params.slope_accel_max_scale);
  node->get_parameter(
    plugin_name_ + ".slope_accel_min_scale",
    params.slope_accel_min_scale);
  node->get_parameter(
    plugin_name_ + ".velocity_smoothing_gain",
    params.velocity_smoothing_gain);
  node->get_parameter(plugin_name_ + ".derivative_step", params.derivative_step);
  int configured_safe_cost = static_cast<int>(params.obstacle_safe_cost);
  node->get_parameter(plugin_name_ + ".obstacle_safe_cost", configured_safe_cost);
  node->get_parameter(plugin_name_ + ".obstacle_weight", params.obstacle_weight);
  node->get_parameter(
    plugin_name_ + ".obstacle_refinement_iterations",
    params.obstacle_refinement_iterations);
  node->get_parameter(
    plugin_name_ + ".obstacle_refinement_gain",
    params.obstacle_refinement_gain);
  node->get_parameter(
    plugin_name_ + ".use_esdf_obstacle_cost",
    params.use_esdf_obstacle_cost);
  node->get_parameter(
    plugin_name_ + ".obstacle_safe_distance",
    params.obstacle_safe_distance);
  node->get_parameter(
    plugin_name_ + ".obstacle_speed_reduction_distance",
    params.obstacle_speed_reduction_distance);
  node->get_parameter(
    plugin_name_ + ".obstacle_speed_min_distance",
    params.obstacle_speed_min_distance);
  node->get_parameter(
    plugin_name_ + ".obstacle_speed_min_scale",
    params.obstacle_speed_min_scale);
  params.obstacle_safe_cost = static_cast<unsigned char>(
    std::max(0, std::min(255, configured_safe_cost)));
  node->get_parameter(plugin_name_ + ".profile_topic", profile_topic_);
  int configured_max_cost = static_cast<int>(max_path_cost_);
  node->get_parameter(plugin_name_ + ".max_path_cost", configured_max_cost);
  int configured_footprint_collision_threshold =
    static_cast<int>(footprint_collision_cost_threshold_);
  node->get_parameter(
    plugin_name_ + ".footprint_collision_cost_threshold",
    configured_footprint_collision_threshold);
  node->get_parameter(plugin_name_ + ".pullback_samples", pullback_samples_);
  node->get_parameter(
    plugin_name_ + ".collision_skip_initial_points", collision_skip_initial_points_);
  node->get_parameter(
    plugin_name_ + ".collision_skip_initial_distance", collision_skip_initial_distance_);
  node->get_parameter(plugin_name_ + ".esdf_source", esdf_source_);
  node->get_parameter(plugin_name_ + ".terrain_pointcloud_topic", terrain_pointcloud_topic_);
  node->get_parameter(plugin_name_ + ".traversability_grid_topic", traversability_grid_topic_);
  node->get_parameter(
    plugin_name_ + ".traversability_height_diff_topic", traversability_height_diff_topic_);
  node->get_parameter(
    plugin_name_ + ".traversability_occupancy_ratio_topic",
    traversability_occupancy_ratio_topic_);
  node->get_parameter(
    plugin_name_ + ".traversability_ground_confidence_topic",
    traversability_ground_confidence_topic_);
  node->get_parameter(
    plugin_name_ + ".traversability_slope_topic", traversability_slope_topic_);
  node->get_parameter(plugin_name_ + ".terrain_esdf_resolution", terrain_esdf_resolution_);
  node->get_parameter(plugin_name_ + ".terrain_esdf_padding", terrain_esdf_padding_);
  node->get_parameter(
    plugin_name_ + ".terrain_esdf_inflation_radius", terrain_esdf_inflation_radius_);
  node->get_parameter(
    plugin_name_ + ".terrain_esdf_min_intensity", terrain_esdf_min_intensity_);
  node->get_parameter(
    plugin_name_ + ".rc_esdf_rolling_window_enabled", rc_esdf_rolling_window_enabled_);
  node->get_parameter(
    plugin_name_ + ".rc_esdf_query_window_size_x", rc_esdf_query_window_size_x_);
  node->get_parameter(
    plugin_name_ + ".rc_esdf_query_window_size_y", rc_esdf_query_window_size_y_);
  node->get_parameter(
    plugin_name_ + ".traversability_slope_max_degrees", traversability_slope_max_degrees_);
  node->get_parameter(
    plugin_name_ + ".traversability_obstacle_value_threshold",
    traversability_obstacle_value_threshold_);
  node->get_parameter(
    plugin_name_ + ".traversability_lethal_value_threshold",
    traversability_lethal_value_threshold_);
  node->get_parameter(
    plugin_name_ + ".traversability_unknown_is_obstacle",
    traversability_unknown_is_obstacle_);
  max_path_cost_ = static_cast<unsigned char>(std::max(0, configured_max_cost));
  footprint_collision_cost_threshold_ = static_cast<unsigned char>(
    std::max(0, std::min(255, configured_footprint_collision_threshold)));
  collision_skip_initial_points_ = std::max(0, collision_skip_initial_points_);
  collision_skip_initial_distance_ = std::max(0.0, collision_skip_initial_distance_);

  optimizer_.setParams(params);
  optimizer_.clearEsdfProvider();
  fake_esdf_provider_ = std::make_shared<FakeCostmapEsdfProvider>();
  terrain_esdf_provider_ = std::make_shared<TerrainPointCloudEsdfProvider>();
  traversability_esdf_provider_ = std::make_shared<RcTraversabilityEsdfProvider>();
  // Configure before subscriptions start delivering data so the first grid update
  // already reflects the intended local-window / slope semantics.
  traversability_esdf_provider_->configureRollingWindow(
    rc_esdf_rolling_window_enabled_,
    rc_esdf_query_window_size_x_,
    rc_esdf_query_window_size_y_);
  traversability_esdf_provider_->setSlopeGridMaxDegrees(traversability_slope_max_degrees_);
  costmap_sub_ = costmap_sub;
  footprint_sub_ = footprint_sub;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  profile_pub_ =
    node->create_publisher<sp_msgs::msg::TrajectoryProfileMsg>(profile_topic_, 10);
  terrain_cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    terrain_pointcloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&Nav2BSplineSmoother::terrainPointCloudCallback, this, std::placeholders::_1));
  traversability_grid_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_grid_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&Nav2BSplineSmoother::traversabilityGridCallback, this, std::placeholders::_1));
  traversability_height_diff_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_height_diff_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &Nav2BSplineSmoother::traversabilityHeightDiffCallback, this, std::placeholders::_1));
  traversability_occupancy_ratio_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_occupancy_ratio_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &Nav2BSplineSmoother::traversabilityOccupancyRatioCallback, this, std::placeholders::_1));
  traversability_ground_confidence_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_ground_confidence_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &Nav2BSplineSmoother::traversabilityGroundConfidenceCallback,
      this, std::placeholders::_1));
  // Pre-subscribe slope semantics here even though the current smoother mainly
  // consumes distance / gradient. This makes later speed-governor integration
  // and debugging much less invasive.
  traversability_slope_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_slope_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &Nav2BSplineSmoother::traversabilitySlopeCallback, this, std::placeholders::_1));
  RCLCPP_INFO(
    logger_, "Configured Nav2BSplineSmoother plugin: %s, esdf_source=%s terrain_topic=%s traversability_topic=%s slope_topic=%s",
    plugin_name_.c_str(), esdf_source_.c_str(), terrain_pointcloud_topic_.c_str(),
    traversability_grid_topic_.c_str(), traversability_slope_topic_.c_str());
}

void Nav2BSplineSmoother::cleanup()
{
}

void Nav2BSplineSmoother::activate()
{
  if (profile_pub_) {
    profile_pub_->on_activate();
  }
}

void Nav2BSplineSmoother::deactivate()
{
  if (profile_pub_) {
    profile_pub_->on_deactivate();
  }
}

bool Nav2BSplineSmoother::smooth(
  nav_msgs::msg::Path & path,
  const rclcpp::Duration &)
{
  const nav_msgs::msg::Path reference_path = path;
  if (costmap_sub_) {
    try {
      const auto costmap = costmap_sub_->getCostmap();
      optimizer_.setObstacleCostmap(costmap);
      const auto params = optimizer_.getParams();
      if (params.use_esdf_obstacle_cost && fake_esdf_provider_ && esdf_source_ == "costmap") {
        fake_esdf_provider_->updateCostmap(costmap, params.obstacle_safe_cost, true);
        active_esdf_provider_ = fake_esdf_provider_;
        RCLCPP_INFO_THROTTLE(
          logger_, *clock_, 5000,
          "Fake ESDF active in Nav2BSplineSmoother: d_safe=%.3f cost_threshold=%d",
          params.obstacle_safe_distance, static_cast<int>(params.obstacle_safe_cost));
      }
    } catch (const std::exception & ex) {
      optimizer_.clearObstacleCostmap();
      if (esdf_source_ == "costmap") {
        active_esdf_provider_.reset();
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Costmap unavailable for bspline smoother, using geometry-only smoothing: %s",
        ex.what());
    }
  } else {
    optimizer_.clearObstacleCostmap();
  }
  refreshEsdfProvider();
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  if (costmap_sub_) {
    try {
      costmap = costmap_sub_->getCostmap();
    } catch (const std::exception &) {
      costmap.reset();
    }
  }
  auto result = optimizer_.optimizeDetailed(path);
  path = result.path;
  enforceCostmapClearance(path, reference_path);
  updatePathOrientations(path);
  if (costmap && pathHasBlockingCollision(*costmap, path)) {
    std::vector<size_t> smoothed_collision_indices;
    collectCollidingIndices(*costmap, path, smoothed_collision_indices);
    if (!smoothed_collision_indices.empty()) {
      const size_t first_idx = smoothed_collision_indices.front();
      unsigned char center_cost = nav2_costmap_2d::NO_INFORMATION;
      const bool has_center_cost = samplePathCost(*costmap, path.poses[first_idx], center_cost);
      const double footprint_cost = sampleFootprintCost(*costmap, path, first_idx);
      const double yaw = estimatePoseYaw(path, first_idx);
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1500,
        "Bspline smoother collision before degradation: count=%zu first_idx=%zu center_cost=%d footprint_cost=%.2f yaw=%.2f",
        smoothed_collision_indices.size(),
        first_idx,
        has_center_cost ? static_cast<int>(center_cost) : -1,
        footprint_cost,
        yaw);
    }

    const bool repaired = locallyDegradeCollidingSegments(*costmap, path, reference_path);
    updatePathOrientations(path);
    if (repaired) {
      enforceCostmapClearance(path, reference_path);
      updatePathOrientations(path);
    }
    if (pathHasBlockingCollision(*costmap, path)) {
      std::vector<size_t> degraded_collision_indices;
      std::vector<size_t> raw_collision_indices;
      collectCollidingIndices(*costmap, path, degraded_collision_indices);
      collectCollidingIndices(*costmap, reference_path, raw_collision_indices);
      if (!degraded_collision_indices.empty()) {
        const size_t first_idx = degraded_collision_indices.front();
        unsigned char center_cost = nav2_costmap_2d::NO_INFORMATION;
        const bool has_center_cost = samplePathCost(*costmap, path.poses[first_idx], center_cost);
        const double footprint_cost = sampleFootprintCost(*costmap, path, first_idx);
        const double yaw = estimatePoseYaw(path, first_idx);
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 1500,
          "Bspline smoother fallback details: degraded_collisions=%zu raw_reference_collisions=%zu first_idx=%zu center_cost=%d footprint_cost=%.2f yaw=%.2f",
          degraded_collision_indices.size(),
          raw_collision_indices.size(),
          first_idx,
          has_center_cost ? static_cast<int>(center_cost) : -1,
          footprint_cost,
          yaw);
      }
      if (raw_collision_indices.size() < degraded_collision_indices.size()) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Smoothed path still collides after local degradation, falling back to raw planner path.");
        path = reference_path;
        updatePathOrientations(path);
      } else {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "Smoothed path still collides after local degradation, but raw planner path is not safer (raw=%zu degraded=%zu). Keep degraded path to avoid reintroducing sharper corners.",
          raw_collision_indices.size(),
          degraded_collision_indices.size());
      }
    }
  }
  result.profile = optimizer_.evaluateProfile(path);
  if (profile_pub_) {
    profile_pub_->publish(
      toProfileMsg(path.header, "nav2_bspline_smoother", result.profile));
  }
  return true;
}

void Nav2BSplineSmoother::terrainPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!terrain_esdf_provider_) {
    return;
  }
  terrain_esdf_provider_->updatePointCloud(
    *msg,
    terrain_esdf_resolution_,
    terrain_esdf_padding_,
    terrain_esdf_inflation_radius_,
    terrain_esdf_min_intensity_);
  if (esdf_source_ == "terrain_pointcloud") {
    active_esdf_provider_ = terrain_esdf_provider_;
    refreshEsdfProvider();
  }
}

void Nav2BSplineSmoother::traversabilityGridCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_grid_msg_ = msg;
  refreshEsdfProvider();
}

void Nav2BSplineSmoother::traversabilityHeightDiffCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_height_diff_msg_ = msg;
  refreshEsdfProvider();
}

void Nav2BSplineSmoother::traversabilityOccupancyRatioCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_occupancy_ratio_msg_ = msg;
  refreshEsdfProvider();
}

void Nav2BSplineSmoother::traversabilityGroundConfidenceCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_ground_confidence_msg_ = msg;
  refreshEsdfProvider();
}

void Nav2BSplineSmoother::traversabilitySlopeCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_slope_msg_ = msg;
  refreshEsdfProvider();
}

void Nav2BSplineSmoother::refreshEsdfProvider()
{
  const auto params = optimizer_.getParams();
  if (!params.use_esdf_obstacle_cost) {
    active_esdf_provider_.reset();
    optimizer_.clearEsdfProvider();
    return;
  }

  if (esdf_source_ == "terrain_pointcloud") {
    if (terrain_esdf_provider_ && terrain_esdf_provider_->available()) {
      active_esdf_provider_ = terrain_esdf_provider_;
      optimizer_.setEsdfProvider(active_esdf_provider_);
    } else {
      optimizer_.clearEsdfProvider();
    }
    return;
  }

  if (esdf_source_ == "traversability_grid") {
    if (traversability_esdf_provider_ && traversability_grid_msg_) {
      // Rebuild from the latest local semantic snapshot each time a side grid arrives.
      // This favors clarity and deterministic behavior over micro-optimizing updates.
      traversability_esdf_provider_->updateGrid(
        *traversability_grid_msg_,
        traversability_obstacle_value_threshold_,
        traversability_unknown_is_obstacle_,
        traversability_lethal_value_threshold_,
        traversability_height_diff_msg_ ? traversability_height_diff_msg_.get() : nullptr,
        traversability_occupancy_ratio_msg_ ? traversability_occupancy_ratio_msg_.get() : nullptr,
        traversability_ground_confidence_msg_ ? traversability_ground_confidence_msg_.get() : nullptr,
        traversability_slope_msg_ ? traversability_slope_msg_.get() : nullptr);
    }
    if (traversability_esdf_provider_ && traversability_esdf_provider_->available()) {
      active_esdf_provider_ = traversability_esdf_provider_;
      optimizer_.setEsdfProvider(active_esdf_provider_);
    } else {
      optimizer_.clearEsdfProvider();
    }
    return;
  }

  if (active_esdf_provider_ && active_esdf_provider_->available()) {
    optimizer_.setEsdfProvider(active_esdf_provider_);
  } else {
    optimizer_.clearEsdfProvider();
  }
}

void Nav2BSplineSmoother::enforceCostmapClearance(
  nav_msgs::msg::Path & smoothed_path,
  const nav_msgs::msg::Path & reference_path) const
{
  if (!costmap_sub_ || smoothed_path.poses.size() < 3 || reference_path.poses.empty()) {
    return;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  try {
    costmap = costmap_sub_->getCostmap();
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "Costmap unavailable for bspline clearance enforcement, skipping pullback: %s",
      ex.what());
    return;
  }
  if (!costmap) {
    RCLCPP_WARN(logger_, "No costmap available for bspline smoother clearance enforcement.");
    return;
  }

  const size_t last_reference_idx = reference_path.poses.size() - 1;
  for (size_t i = 1; i + 1 < smoothed_path.poses.size(); ++i) {
    unsigned char cost = nav2_costmap_2d::NO_INFORMATION;
    const bool sampled_center_cost = samplePathCost(*costmap, smoothed_path.poses[i], cost);
    const double footprint_cost = sampleFootprintCost(*costmap, smoothed_path, i);
    const bool footprint_collision = footprint_cost > footprint_collision_cost_threshold_;
    if (!sampled_center_cost && footprint_cost < 0.0) {
      continue;
    }
    if (!footprint_collision && sampled_center_cost && cost <= max_path_cost_) {
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
    auto best_candidate = current_pose;
    unsigned char best_center_cost = sampled_center_cost ? cost : nav2_costmap_2d::NO_INFORMATION;
    double best_footprint_cost = footprint_cost;
    for (int step = 1; step <= pullback_samples_; ++step) {
      const double alpha = static_cast<double>(step) / static_cast<double>(pullback_samples_);
      auto candidate = projectTowardReference(current_pose, original_pose, alpha);

      unsigned char candidate_cost = nav2_costmap_2d::NO_INFORMATION;
      const bool sampled_candidate_cost = samplePathCost(*costmap, candidate, candidate_cost);
      const double candidate_footprint_cost =
        sampleFootprintCost(*costmap, smoothed_path, i, &candidate);
      const bool candidate_footprint_collision =
        candidate_footprint_cost > footprint_collision_cost_threshold_;
      if (!sampled_candidate_cost && candidate_footprint_cost < 0.0) {
        continue;
      }

      const bool better_footprint =
        candidate_footprint_cost >= 0.0 &&
        (best_footprint_cost < 0.0 || candidate_footprint_cost < best_footprint_cost);
      const bool same_footprint_better_center =
        candidate_footprint_cost == best_footprint_cost &&
        sampled_candidate_cost &&
        (!sampled_center_cost || candidate_cost < best_center_cost);
      if (better_footprint || same_footprint_better_center) {
        best_footprint_cost = candidate_footprint_cost;
        best_center_cost = candidate_cost;
        best_candidate = candidate;
      }

      if (!candidate_footprint_collision && sampled_candidate_cost &&
        candidate_cost <= max_path_cost_)
      {
        smoothed_path.poses[i] = candidate;
        repaired = true;
        break;
      }
    }

    if (!repaired) {
      smoothed_path.poses[i] = best_candidate;
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

double Nav2BSplineSmoother::estimatePoseYaw(
  const nav_msgs::msg::Path & path,
  size_t index,
  const geometry_msgs::msg::PoseStamped * override_pose) const
{
  const auto & current_pose = override_pose ? *override_pose : path.poses[index];
  if (path.poses.size() < 2) {
    return tf2::getYaw(current_pose.pose.orientation);
  }

  geometry_msgs::msg::Point previous_point;
  geometry_msgs::msg::Point next_point;
  if (index == 0) {
    previous_point = current_pose.pose.position;
    next_point = path.poses[1].pose.position;
  } else if (index + 1 >= path.poses.size()) {
    previous_point = path.poses[index - 1].pose.position;
    next_point = current_pose.pose.position;
  } else {
    previous_point = path.poses[index - 1].pose.position;
    next_point = path.poses[index + 1].pose.position;
  }

  const double dx = next_point.x - previous_point.x;
  const double dy = next_point.y - previous_point.y;
  if (std::abs(dx) <= 1e-9 && std::abs(dy) <= 1e-9) {
    return tf2::getYaw(current_pose.pose.orientation);
  }
  return std::atan2(dy, dx);
}

bool Nav2BSplineSmoother::getRobotFootprint(nav2_costmap_2d::Footprint & footprint) const
{
  if (!footprint_sub_) {
    return false;
  }

  std_msgs::msg::Header footprint_header;
  return footprint_sub_->getFootprintInRobotFrame(footprint, footprint_header);
}

double Nav2BSplineSmoother::sampleFootprintCost(
  nav2_costmap_2d::Costmap2D & costmap,
  const nav_msgs::msg::Path & path,
  size_t index,
  const geometry_msgs::msg::PoseStamped * override_pose) const
{
  nav2_costmap_2d::Footprint footprint;
  if (!getRobotFootprint(footprint) || footprint.size() < 3) {
    return -1.0;
  }

  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(&costmap);
  const auto & pose = override_pose ? *override_pose : path.poses[index];
  const double tangent_yaw = estimatePoseYaw(path, index, override_pose);
  const double pose_yaw = tf2::getYaw(pose.pose.orientation);

  // For the omni sentry chain, path tangent is not the only feasible body yaw.
  // Checking a few equivalent candidate headings reduces false collisions where
  // position is valid but a tangent-aligned square footprint clips a corner.
  const std::array<double, 5> candidate_yaws = {
    tangent_yaw,
    pose_yaw,
    tangent_yaw + kHalfPi,
    tangent_yaw - kHalfPi,
    tangent_yaw + kPi};

  double best_cost = -1.0;
  for (const double yaw : candidate_yaws) {
    const double cost = checker.footprintCostAtPose(
      pose.pose.position.x,
      pose.pose.position.y,
      yaw,
      footprint);
    if (cost < 0.0) {
      continue;
    }
    if (best_cost < 0.0 || cost < best_cost) {
      best_cost = cost;
    }
  }

  return best_cost;
}

void Nav2BSplineSmoother::collectCollidingIndices(
  nav2_costmap_2d::Costmap2D & costmap,
  const nav_msgs::msg::Path & path,
  std::vector<size_t> & indices) const
{
  indices.clear();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double footprint_cost = sampleFootprintCost(costmap, path, i);
    if (footprint_cost > footprint_collision_cost_threshold_) {
      indices.push_back(i);
    }
  }
}

bool Nav2BSplineSmoother::pathHasBlockingCollision(
  nav2_costmap_2d::Costmap2D & costmap,
  const nav_msgs::msg::Path & path) const
{
  std::vector<size_t> indices;
  collectCollidingIndices(costmap, path, indices);
  if (indices.empty()) {
    return false;
  }

  const auto & start = path.poses.front().pose.position;
  for (const size_t index : indices) {
    const auto & point = path.poses[index].pose.position;
    const double dx = point.x - start.x;
    const double dy = point.y - start.y;
    const double distance_from_start = std::hypot(dx, dy);
    const bool within_startup_index =
      index < static_cast<size_t>(collision_skip_initial_points_);
    const bool within_startup_distance =
      distance_from_start <= collision_skip_initial_distance_;
    if (!within_startup_index && !within_startup_distance) {
      return true;
    }
  }

  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 1500,
    "Ignoring %zu startup footprint collision samples within %.2fm / %d points; keep controller active for MPPI to move out of local self-cost.",
    indices.size(),
    collision_skip_initial_distance_,
    collision_skip_initial_points_);
  return false;
}

bool Nav2BSplineSmoother::locallyDegradeCollidingSegments(
  nav2_costmap_2d::Costmap2D & costmap,
  nav_msgs::msg::Path & path,
  const nav_msgs::msg::Path & reference_path) const
{
  if (path.poses.size() < 3 || reference_path.poses.size() < 3) {
    return false;
  }

  std::vector<size_t> collision_indices;
  collectCollidingIndices(costmap, path, collision_indices);
  if (collision_indices.empty()) {
    return false;
  }

  const size_t path_last = path.poses.size() - 1;
  const size_t ref_last = reference_path.poses.size() - 1;
  const size_t window_radius = std::max<size_t>(2, static_cast<size_t>(pullback_samples_ / 2));

  for (const size_t collision_index : collision_indices) {
    const size_t window_start = (collision_index > window_radius) ?
      (collision_index - window_radius) : 0;
    const size_t window_end = std::min(path_last, collision_index + window_radius);

    for (size_t i = window_start; i <= window_end; ++i) {
      const double ratio =
        static_cast<double>(i) / static_cast<double>(std::max<size_t>(1, path_last));
      const size_t ref_idx = std::min(
        ref_last,
        static_cast<size_t>(std::round(ratio * static_cast<double>(ref_last))));
      path.poses[i].pose.position = reference_path.poses[ref_idx].pose.position;
    }
  }

  return true;
}

void Nav2BSplineSmoother::updatePathOrientations(nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {
    return;
  }

  double yaw = 0.0;
  for (size_t i = 0; i < path.poses.size(); ++i) {
    if (i + 1 < path.poses.size()) {
      yaw = std::atan2(
        path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y,
        path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x);
    }
    path.poses[i].pose.orientation.z = std::sin(0.5 * yaw);
    path.poses[i].pose.orientation.w = std::cos(0.5 * yaw);
  }
}

geometry_msgs::msg::PoseStamped Nav2BSplineSmoother::projectTowardReference(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::PoseStamped & reference_pose,
  double step_ratio) const
{
  auto candidate = current_pose;
  const double clamped_ratio = std::max(0.0, std::min(1.0, step_ratio));
  candidate.pose.position.x =
    current_pose.pose.position.x +
    (reference_pose.pose.position.x - current_pose.pose.position.x) * clamped_ratio;
  candidate.pose.position.y =
    current_pose.pose.position.y +
    (reference_pose.pose.position.y - current_pose.pose.position.y) * clamped_ratio;
  return candidate;
}

}  // namespace trajectory_optimizer

PLUGINLIB_EXPORT_CLASS(trajectory_optimizer::Nav2BSplineSmoother, nav2_core::Smoother)
