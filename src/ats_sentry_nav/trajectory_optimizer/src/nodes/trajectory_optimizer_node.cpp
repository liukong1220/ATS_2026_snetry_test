// Copyright 2026

#include "trajectory_optimizer/nodes/trajectory_optimizer_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std::string formatDouble(double value)
{
  if (!std::isfinite(value)) {
    return "n/a";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}

geometry_msgs::msg::Point offsetPoint(
  const geometry_msgs::msg::Point & origin,
  double dx,
  double dy,
  double dz)
{
  geometry_msgs::msg::Point shifted = origin;
  shifted.x += dx;
  shifted.y += dy;
  shifted.z += dz;
  return shifted;
}

template<typename PublisherT>
bool hasSubscribers(const std::shared_ptr<PublisherT> & publisher)
{
  return publisher &&
         (publisher->get_subscription_count() > 0 ||
         publisher->get_intra_process_subscription_count() > 0);
}

}  // namespace

TrajectoryOptimizerNode::TrajectoryOptimizerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_optimizer", options)
{
  declare_parameter<std::string>("input_path_topic", "plan");
  declare_parameter<std::string>("output_path_topic", "smoothed_path");
  declare_parameter<std::string>("output_profile_topic", "trajectory_profile_visual");
  declare_parameter<std::string>("costmap_topic", "global_costmap/costmap_raw");
  declare_parameter<double>("control_point_spacing", params_.control_point_spacing);
  declare_parameter<double>("output_path_spacing", params_.output_path_spacing);
  declare_parameter<double>("min_input_point_spacing", params_.min_input_point_spacing);
  declare_parameter<double>("max_lateral_deviation", params_.max_lateral_deviation);
  declare_parameter<int>("min_control_points", params_.min_control_points);
  declare_parameter<bool>("use_continuous_optimization", params_.use_continuous_optimization);
  declare_parameter<int>("continuous_max_iterations", params_.continuous_max_iterations);
  declare_parameter<int>("continuous_lbfgs_memory", params_.continuous_lbfgs_memory);
  declare_parameter<double>(
    "continuous_gradient_tolerance", params_.continuous_gradient_tolerance);
  declare_parameter<double>("continuous_initial_step", params_.continuous_initial_step);
  declare_parameter<double>("smoothness_weight", params_.smoothness_weight);
  declare_parameter<double>("fitness_weight", params_.fitness_weight);
  declare_parameter<double>("endpoint_tangent_weight", params_.endpoint_tangent_weight);
  declare_parameter<double>("corridor_weight", params_.corridor_weight);
  declare_parameter<double>("curvature_limit", params_.curvature_limit);
  declare_parameter<double>("curvature_weight", params_.curvature_weight);
  declare_parameter<int>(
    "curvature_refinement_iterations", params_.curvature_refinement_iterations);
  declare_parameter<double>("curvature_refinement_gain", params_.curvature_refinement_gain);
  declare_parameter<double>("global_speed_limit", params_.global_speed_limit);
  declare_parameter<double>("lateral_accel_limit", params_.lateral_accel_limit);
  declare_parameter<double>("longitudinal_accel_limit", params_.longitudinal_accel_limit);
  declare_parameter<bool>("use_slope_speed_limits", params_.use_slope_speed_limits);
  declare_parameter<double>("slope_speed_boost_start_deg", params_.slope_speed_boost_start_deg);
  declare_parameter<double>("slope_speed_obstacle_deg", params_.slope_speed_obstacle_deg);
  declare_parameter<double>("slope_speed_limit_full_deg", params_.slope_speed_limit_full_deg);
  declare_parameter<double>("slope_speed_max_scale", params_.slope_speed_max_scale);
  declare_parameter<double>("slope_speed_min_scale", params_.slope_speed_min_scale);
  declare_parameter<bool>("use_slope_accel_limits", params_.use_slope_accel_limits);
  declare_parameter<double>("slope_accel_boost_start_deg", params_.slope_accel_boost_start_deg);
  declare_parameter<double>("slope_accel_obstacle_deg", params_.slope_accel_obstacle_deg);
  declare_parameter<double>("slope_accel_limit_full_deg", params_.slope_accel_limit_full_deg);
  declare_parameter<double>("slope_accel_max_scale", params_.slope_accel_max_scale);
  declare_parameter<double>("slope_accel_min_scale", params_.slope_accel_min_scale);
  declare_parameter<double>("velocity_smoothing_gain", params_.velocity_smoothing_gain);
  declare_parameter<double>("derivative_step", params_.derivative_step);
  declare_parameter<int>("obstacle_safe_cost", static_cast<int>(params_.obstacle_safe_cost));
  declare_parameter<double>("obstacle_weight", params_.obstacle_weight);
  declare_parameter<int>(
    "obstacle_refinement_iterations", params_.obstacle_refinement_iterations);
  declare_parameter<double>("obstacle_refinement_gain", params_.obstacle_refinement_gain);
  declare_parameter<bool>("use_esdf_obstacle_cost", params_.use_esdf_obstacle_cost);
  declare_parameter<double>("obstacle_safe_distance", params_.obstacle_safe_distance);
  declare_parameter<std::string>("esdf_debug_topic", esdf_debug_topic_);
  declare_parameter<std::string>("esdf_source", esdf_source_);
  declare_parameter<std::string>("terrain_pointcloud_topic", terrain_pointcloud_topic_);
  // The traversability-grid ESDF path is the V1 task-1 bridge from 2.5D semantics
  // into the existing smoother / optimizer chain.
  declare_parameter<std::string>("traversability_grid_topic", traversability_grid_topic_);
  declare_parameter<std::string>(
    "traversability_height_diff_topic", traversability_height_diff_topic_);
  declare_parameter<std::string>(
    "traversability_occupancy_ratio_topic", traversability_occupancy_ratio_topic_);
  declare_parameter<std::string>(
    "traversability_ground_confidence_topic", traversability_ground_confidence_topic_);
  declare_parameter<std::string>("traversability_slope_topic", traversability_slope_topic_);
  declare_parameter<double>("terrain_esdf_resolution", terrain_esdf_resolution_);
  declare_parameter<double>("terrain_esdf_padding", terrain_esdf_padding_);
  declare_parameter<double>("terrain_esdf_inflation_radius", terrain_esdf_inflation_radius_);
  declare_parameter<double>("terrain_esdf_min_intensity", terrain_esdf_min_intensity_);
  // RC-ESDF-lite parameters:
  // - rolling_window_enabled controls whether queries are restricted to an explicit local box
  // - query_window_size_x / y let us expose local-query intent without changing map transport
  // - slope_max_degrees decodes 0~100 slope_grid values back into physical degrees
  declare_parameter<bool>("rc_esdf_rolling_window_enabled", rc_esdf_rolling_window_enabled_);
  declare_parameter<double>("rc_esdf_query_window_size_x", rc_esdf_query_window_size_x_);
  declare_parameter<double>("rc_esdf_query_window_size_y", rc_esdf_query_window_size_y_);
  declare_parameter<double>("traversability_slope_max_degrees", traversability_slope_max_degrees_);
  declare_parameter<int>(
    "traversability_obstacle_value_threshold", traversability_obstacle_value_threshold_);
  declare_parameter<int>(
    "traversability_lethal_value_threshold", traversability_lethal_value_threshold_);
  declare_parameter<bool>(
    "traversability_unknown_is_obstacle", traversability_unknown_is_obstacle_);

  get_parameter("input_path_topic", input_path_topic_);
  get_parameter("output_path_topic", output_path_topic_);
  get_parameter("output_profile_topic", output_profile_topic_);
  get_parameter("costmap_topic", costmap_topic_);
  get_parameter("control_point_spacing", params_.control_point_spacing);
  get_parameter("output_path_spacing", params_.output_path_spacing);
  get_parameter("min_input_point_spacing", params_.min_input_point_spacing);
  get_parameter("max_lateral_deviation", params_.max_lateral_deviation);
  get_parameter("min_control_points", params_.min_control_points);
  get_parameter("use_continuous_optimization", params_.use_continuous_optimization);
  get_parameter("continuous_max_iterations", params_.continuous_max_iterations);
  get_parameter("continuous_lbfgs_memory", params_.continuous_lbfgs_memory);
  get_parameter(
    "continuous_gradient_tolerance", params_.continuous_gradient_tolerance);
  get_parameter("continuous_initial_step", params_.continuous_initial_step);
  get_parameter("smoothness_weight", params_.smoothness_weight);
  get_parameter("fitness_weight", params_.fitness_weight);
  get_parameter("endpoint_tangent_weight", params_.endpoint_tangent_weight);
  get_parameter("corridor_weight", params_.corridor_weight);
  get_parameter("curvature_limit", params_.curvature_limit);
  get_parameter("curvature_weight", params_.curvature_weight);
  get_parameter("curvature_refinement_iterations", params_.curvature_refinement_iterations);
  get_parameter("curvature_refinement_gain", params_.curvature_refinement_gain);
  get_parameter("global_speed_limit", params_.global_speed_limit);
  get_parameter("lateral_accel_limit", params_.lateral_accel_limit);
  get_parameter("longitudinal_accel_limit", params_.longitudinal_accel_limit);
  get_parameter("use_slope_speed_limits", params_.use_slope_speed_limits);
  get_parameter("slope_speed_boost_start_deg", params_.slope_speed_boost_start_deg);
  get_parameter("slope_speed_obstacle_deg", params_.slope_speed_obstacle_deg);
  get_parameter("slope_speed_limit_full_deg", params_.slope_speed_limit_full_deg);
  get_parameter("slope_speed_max_scale", params_.slope_speed_max_scale);
  get_parameter("slope_speed_min_scale", params_.slope_speed_min_scale);
  get_parameter("use_slope_accel_limits", params_.use_slope_accel_limits);
  get_parameter("slope_accel_boost_start_deg", params_.slope_accel_boost_start_deg);
  get_parameter("slope_accel_obstacle_deg", params_.slope_accel_obstacle_deg);
  get_parameter("slope_accel_limit_full_deg", params_.slope_accel_limit_full_deg);
  get_parameter("slope_accel_max_scale", params_.slope_accel_max_scale);
  get_parameter("slope_accel_min_scale", params_.slope_accel_min_scale);
  get_parameter("velocity_smoothing_gain", params_.velocity_smoothing_gain);
  get_parameter("derivative_step", params_.derivative_step);
  int configured_safe_cost = static_cast<int>(params_.obstacle_safe_cost);
  get_parameter("obstacle_safe_cost", configured_safe_cost);
  get_parameter("obstacle_weight", params_.obstacle_weight);
  get_parameter("obstacle_refinement_iterations", params_.obstacle_refinement_iterations);
  get_parameter("obstacle_refinement_gain", params_.obstacle_refinement_gain);
  get_parameter("use_esdf_obstacle_cost", params_.use_esdf_obstacle_cost);
  get_parameter("obstacle_safe_distance", params_.obstacle_safe_distance);
  get_parameter("esdf_debug_topic", esdf_debug_topic_);
  get_parameter("esdf_source", esdf_source_);
  get_parameter("terrain_pointcloud_topic", terrain_pointcloud_topic_);
  get_parameter("traversability_grid_topic", traversability_grid_topic_);
  get_parameter("traversability_height_diff_topic", traversability_height_diff_topic_);
  get_parameter(
    "traversability_occupancy_ratio_topic", traversability_occupancy_ratio_topic_);
  get_parameter(
    "traversability_ground_confidence_topic", traversability_ground_confidence_topic_);
  get_parameter("traversability_slope_topic", traversability_slope_topic_);
  get_parameter("terrain_esdf_resolution", terrain_esdf_resolution_);
  get_parameter("terrain_esdf_padding", terrain_esdf_padding_);
  get_parameter("terrain_esdf_inflation_radius", terrain_esdf_inflation_radius_);
  get_parameter("terrain_esdf_min_intensity", terrain_esdf_min_intensity_);
  get_parameter("rc_esdf_rolling_window_enabled", rc_esdf_rolling_window_enabled_);
  get_parameter("rc_esdf_query_window_size_x", rc_esdf_query_window_size_x_);
  get_parameter("rc_esdf_query_window_size_y", rc_esdf_query_window_size_y_);
  get_parameter("traversability_slope_max_degrees", traversability_slope_max_degrees_);
  get_parameter(
    "traversability_obstacle_value_threshold", traversability_obstacle_value_threshold_);
  get_parameter(
    "traversability_lethal_value_threshold", traversability_lethal_value_threshold_);
  get_parameter(
    "traversability_unknown_is_obstacle", traversability_unknown_is_obstacle_);
  params_.obstacle_safe_cost = static_cast<unsigned char>(
    std::max(0, std::min(255, configured_safe_cost)));
  optimizer_.setParams(params_);
  fake_esdf_provider_ = std::make_shared<FakeCostmapEsdfProvider>();
  terrain_esdf_provider_ = std::make_shared<TerrainPointCloudEsdfProvider>();
  traversability_esdf_provider_ = std::make_shared<RcTraversabilityEsdfProvider>();
  // Configure the provider once here so every subsequent grid update reuses the same
  // policy about local-window bounds and slope decoding.
  traversability_esdf_provider_->configureRollingWindow(
    rc_esdf_rolling_window_enabled_,
    rc_esdf_query_window_size_x_,
    rc_esdf_query_window_size_y_);
  traversability_esdf_provider_->setSlopeGridMaxDegrees(traversability_slope_max_degrees_);
  optimizer_.clearEsdfProvider();

  smoothed_path_pub_ = create_publisher<nav_msgs::msg::Path>(output_path_topic_, 10);
  profile_pub_ =
    create_publisher<sp_msgs::msg::TrajectoryProfileMsg>(output_profile_topic_, 10);
  esdf_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(esdf_debug_topic_, 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    input_path_topic_, 10,
    std::bind(&TrajectoryOptimizerNode::pathCallback, this, std::placeholders::_1));
  terrain_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    terrain_pointcloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TrajectoryOptimizerNode::terrainPointCloudCallback, this, std::placeholders::_1));
  traversability_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_grid_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&TrajectoryOptimizerNode::traversabilityGridCallback, this, std::placeholders::_1));
  traversability_height_diff_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_height_diff_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &TrajectoryOptimizerNode::traversabilityHeightDiffCallback, this, std::placeholders::_1));
  traversability_occupancy_ratio_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_occupancy_ratio_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &TrajectoryOptimizerNode::traversabilityOccupancyRatioCallback,
      this, std::placeholders::_1));
  traversability_ground_confidence_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_ground_confidence_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &TrajectoryOptimizerNode::traversabilityGroundConfidenceCallback,
      this, std::placeholders::_1));
  // slope_grid is not yet used for speed control in task 1, but subscribing now keeps
  // the ESDF-side semantic API stable before task 2 touches the governor.
  traversability_slope_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    traversability_slope_topic_, rclcpp::QoS(10).reliable(),
    std::bind(
      &TrajectoryOptimizerNode::traversabilitySlopeCallback,
      this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Trajectory optimizer active: %s -> %s, esdf_source=%s terrain_topic=%s traversability_topic=%s slope_topic=%s",
    input_path_topic_.c_str(), output_path_topic_.c_str(), esdf_source_.c_str(),
    terrain_pointcloud_topic_.c_str(), traversability_grid_topic_.c_str(),
    traversability_slope_topic_.c_str());
}

void TrajectoryOptimizerNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  const bool publish_smoothed_path = hasSubscribers(smoothed_path_pub_);
  const bool publish_profile = hasSubscribers(profile_pub_);
  const bool publish_esdf_debug = hasSubscribers(esdf_marker_pub_);
  if (!publish_smoothed_path && !publish_profile && !publish_esdf_debug) {
    return;
  }

  if (!costmap_sub_) {
    costmap_sub_ =
      std::make_shared<nav2_costmap_2d::CostmapSubscriber>(shared_from_this(), costmap_topic_);
  }
  if (costmap_sub_) {
    try {
      const auto costmap = costmap_sub_->getCostmap();
      optimizer_.setObstacleCostmap(costmap);
      if (params_.use_esdf_obstacle_cost && fake_esdf_provider_ && esdf_source_ == "costmap") {
        fake_esdf_provider_->updateCostmap(costmap, params_.obstacle_safe_cost, true);
        active_esdf_provider_ = fake_esdf_provider_;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Fake ESDF active in trajectory_optimizer_node: d_safe=%.3f cost_threshold=%d",
          params_.obstacle_safe_distance, static_cast<int>(params_.obstacle_safe_cost));
      }
    } catch (const std::exception & ex) {
      optimizer_.clearObstacleCostmap();
      active_esdf_provider_.reset();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Costmap unavailable for visual trajectory optimizer, using geometry-only path: %s",
        ex.what());
    }
  }
  refreshEsdfProvider();
  const auto result = optimizer_.optimizeDetailed(*msg);
  if (publish_smoothed_path) {
    smoothed_path_pub_->publish(result.path);
  }
  if (publish_profile) {
    profile_pub_->publish(toProfileMsg(msg->header, "trajectory_optimizer_node", result.profile));
  }
  if (publish_esdf_debug) {
    publishEsdfDebugMarkers(result.path);
  }
}

void TrajectoryOptimizerNode::terrainPointCloudCallback(
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

void TrajectoryOptimizerNode::traversabilityGridCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_grid_msg_ = msg;
  updateTraversabilityEsdf();
}

void TrajectoryOptimizerNode::traversabilityHeightDiffCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_height_diff_msg_ = msg;
  updateTraversabilityEsdf();
}

void TrajectoryOptimizerNode::traversabilityOccupancyRatioCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_occupancy_ratio_msg_ = msg;
  updateTraversabilityEsdf();
}

void TrajectoryOptimizerNode::traversabilityGroundConfidenceCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_ground_confidence_msg_ = msg;
  updateTraversabilityEsdf();
}

void TrajectoryOptimizerNode::traversabilitySlopeCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  traversability_slope_msg_ = msg;
  updateTraversabilityEsdf();
}

void TrajectoryOptimizerNode::updateTraversabilityEsdf()
{
  if (!traversability_esdf_provider_ || !traversability_grid_msg_) {
    return;
  }

  // The traversability grid is the required backbone; other semantic grids are optional
  // enrichments. If a side grid is absent or misaligned, updateGrid will gracefully
  // keep ESDF alive and mark the missing semantic channel as unavailable.
  traversability_esdf_provider_->updateGrid(
    *traversability_grid_msg_,
    traversability_obstacle_value_threshold_,
    traversability_unknown_is_obstacle_,
    traversability_lethal_value_threshold_,
    traversability_height_diff_msg_ ? traversability_height_diff_msg_.get() : nullptr,
    traversability_occupancy_ratio_msg_ ? traversability_occupancy_ratio_msg_.get() : nullptr,
    traversability_ground_confidence_msg_ ? traversability_ground_confidence_msg_.get() : nullptr,
    traversability_slope_msg_ ? traversability_slope_msg_.get() : nullptr);
  if (esdf_source_ == "traversability_grid") {
    active_esdf_provider_ = traversability_esdf_provider_;
    refreshEsdfProvider();
  }
}

void TrajectoryOptimizerNode::refreshEsdfProvider()
{
  if (!params_.use_esdf_obstacle_cost) {
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

void TrajectoryOptimizerNode::publishEsdfDebugMarkers(const nav_msgs::msg::Path & path)
{
  if (!esdf_marker_pub_ || !active_esdf_provider_ || !params_.use_esdf_obstacle_cost ||
    !active_esdf_provider_->available() || path.poses.empty() ||
    !hasSubscribers(esdf_marker_pub_))
  {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header = path.header;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker distance_points;
  distance_points.header = path.header;
  distance_points.ns = "trajectory_esdf";
  distance_points.id = 0;
  distance_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  distance_points.action = visualization_msgs::msg::Marker::ADD;
  distance_points.scale.x = 0.045;
  distance_points.scale.y = 0.045;
  distance_points.scale.z = 0.045;

  visualization_msgs::msg::Marker gradient_arrows;
  gradient_arrows.header = path.header;
  gradient_arrows.ns = "trajectory_esdf";
  gradient_arrows.id = 1;
  gradient_arrows.type = visualization_msgs::msg::Marker::ARROW;
  gradient_arrows.action = visualization_msgs::msg::Marker::ADD;
  gradient_arrows.scale.x = 0.016;
  gradient_arrows.scale.y = 0.028;
  gradient_arrows.scale.z = 0.038;

  visualization_msgs::msg::Marker text;
  text.header = path.header;
  text.ns = "trajectory_esdf";
  text.id = 2;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.scale.z = 0.14;
  text.color = makeColor(0.90f, 0.90f, 0.86f, 0.88f);
  text.pose.orientation.w = 1.0;

  double min_distance = std::numeric_limits<double>::infinity();
  double avg_distance = 0.0;
  double avg_gradient_norm = 0.0;
  std::size_t valid_distance_count = 0;
  std::size_t danger_count = 0;
  std::size_t gradient_count = 0;
  for (const auto & pose : path.poses) {
    const double distance = active_esdf_provider_->getDistance(
      pose.pose.position.x, pose.pose.position.y);
    const Eigen::Vector2d gradient = active_esdf_provider_->getGradient(
      pose.pose.position.x, pose.pose.position.y);
    const double gradient_norm = gradient.norm();

    distance_points.points.push_back(pose.pose.position);
    const double ratio = std::max(0.0, std::min(1.0, distance / std::max(0.05, params_.obstacle_safe_distance)));
    distance_points.colors.push_back(
      makeColor(
        static_cast<float>(0.78 - 0.38 * ratio),
        static_cast<float>(0.32 + 0.42 * ratio),
        static_cast<float>(0.26 + 0.08 * ratio),
        0.82f));

    geometry_msgs::msg::Point start = pose.pose.position;
    geometry_msgs::msg::Point end = start;
    end.x += gradient.x() * 0.12;
    end.y += gradient.y() * 0.12;
    gradient_arrows.points.clear();
    gradient_arrows.colors.clear();
    gradient_arrows.id = static_cast<int>(10 + gradient_count);
    gradient_arrows.points.push_back(start);
    gradient_arrows.points.push_back(end);
    const double grad_ratio = std::max(0.0, std::min(1.0, gradient_norm / 2.0));
    gradient_arrows.color = makeColor(
      static_cast<float>(0.86),
      static_cast<float>(0.70 - 0.22 * grad_ratio),
      static_cast<float>(0.30),
      0.80f);
    markers.markers.push_back(gradient_arrows);

    if (std::isfinite(distance) && distance >= 0.0) {
      min_distance = std::min(min_distance, distance);
      avg_distance += distance;
      ++valid_distance_count;
      if (distance < params_.obstacle_safe_distance) {
        ++danger_count;
      }
    }
    if (std::isfinite(gradient_norm)) {
      avg_gradient_norm += gradient_norm;
      ++gradient_count;
    }
  }

  avg_distance = valid_distance_count > 0 ?
    (avg_distance / static_cast<double>(valid_distance_count)) : 0.0;
  avg_gradient_norm = gradient_count > 0 ?
    (avg_gradient_norm / static_cast<double>(gradient_count)) : 0.0;
  const auto & tail = path.poses.back().pose.position;
  double text_dx = 0.0;
  double text_dy = 0.0;
  if (path.poses.size() >= 2) {
    const auto & prev = path.poses[path.poses.size() - 2].pose.position;
    const double tangent_x = tail.x - prev.x;
    const double tangent_y = tail.y - prev.y;
    const double tangent_norm = std::hypot(tangent_x, tangent_y);
    if (tangent_norm > 1e-6) {
      const double normal_x = -tangent_y / tangent_norm;
      const double normal_y = tangent_x / tangent_norm;
      text_dx = normal_x * 0.28;
      text_dy = normal_y * 0.28;
    } else {
      text_dx = -0.18;
      text_dy = 0.18;
    }
  } else {
    text_dx = -0.18;
    text_dy = 0.18;
  }
  text.pose.position = offsetPoint(tail, text_dx, text_dy, 0.46);
  text.text =
    "d_min=" + formatDouble(min_distance) +
    " d_avg=" + formatDouble(avg_distance) +
    " |g|=" + formatDouble(avg_gradient_norm) +
    " risk=" + std::to_string(danger_count) + "/" + std::to_string(valid_distance_count);

  markers.markers.push_back(distance_points);
  markers.markers.push_back(text);
  esdf_marker_pub_->publish(markers);
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 3000,
    "ESDF debug stats: valid=%zu danger=%zu d_min=%s d_avg=%s |grad|_avg=%s",
    valid_distance_count,
    danger_count,
    formatDouble(min_distance).c_str(),
    formatDouble(avg_distance).c_str(),
    formatDouble(avg_gradient_norm).c_str());
}

}  // namespace trajectory_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimizer::TrajectoryOptimizerNode)
