// Copyright 2026

#include "minco_planner/nodes/minco_planner_node.hpp"

#include <algorithm>
#include <cmath>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace minco_planner
{

MincoPlannerNode::MincoPlannerNode(const rclcpp::NodeOptions & options)
: Node("minco_planner", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  declareAndLoadParams();

  grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    grid_topic_, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&MincoPlannerNode::onGrid, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(10),
    std::bind(&MincoPlannerNode::onGoal, this, std::placeholders::_1));
  raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(raw_path_topic_, rclcpp::QoS(1));
  reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    reference_path_topic_, rclcpp::QoS(1));
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    debug_marker_topic_, rclcpp::QoS(1));

  RCLCPP_INFO(
    get_logger(),
    "minco_planner ready: grid='%s' goal='%s' raw='%s' reference='%s'",
    grid_topic_.c_str(),
    goal_topic_.c_str(),
    raw_path_topic_.c_str(),
    reference_path_topic_.c_str());
}

void MincoPlannerNode::declareAndLoadParams()
{
  declare_parameter<std::string>("grid_topic", grid_topic_);
  declare_parameter<std::string>("goal_topic", goal_topic_);
  declare_parameter<std::string>("raw_path_topic", raw_path_topic_);
  declare_parameter<std::string>("reference_path_topic", reference_path_topic_);
  declare_parameter<std::string>("debug_marker_topic", debug_marker_topic_);
  declare_parameter<std::string>("global_frame", global_frame_);
  declare_parameter<std::string>("robot_frame", robot_frame_);

  GridAstarParams astar_params;
  declare_parameter<int>("obstacle_value_threshold", astar_params.obstacle_value_threshold);
  declare_parameter<bool>("unknown_is_obstacle", astar_params.unknown_is_obstacle);
  declare_parameter<bool>("allow_diagonal", astar_params.allow_diagonal);

  MincoTrajectoryOptimizerParams optimizer_params;
  declare_parameter<double>("reference_speed", optimizer_params.reference_speed);
  declare_parameter<double>("min_segment_time", optimizer_params.min_segment_time);
  declare_parameter<double>("sample_spacing", optimizer_params.sample_spacing);

  YawSplinePlannerParams yaw_params;
  declare_parameter<double>("yaw_rate_limit", yaw_params.yaw_rate_limit);

  FootprintSafetyParams footprint_params;
  declare_parameter<double>("footprint_length", footprint_params.length);
  declare_parameter<double>("footprint_width", footprint_params.width);
  declare_parameter<double>("footprint_safety_margin", footprint_params.safety_margin);

  LocalCollisionRepairParams repair_params;
  declare_parameter<bool>("local_repair_enabled", repair_params.enabled);
  declare_parameter<int>("local_repair_max_iterations", repair_params.max_iterations);
  declare_parameter<double>("local_repair_search_radius", repair_params.search_radius);

  get_parameter("grid_topic", grid_topic_);
  get_parameter("goal_topic", goal_topic_);
  get_parameter("raw_path_topic", raw_path_topic_);
  get_parameter("reference_path_topic", reference_path_topic_);
  get_parameter("debug_marker_topic", debug_marker_topic_);
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("obstacle_value_threshold", astar_params.obstacle_value_threshold);
  get_parameter("unknown_is_obstacle", astar_params.unknown_is_obstacle);
  get_parameter("allow_diagonal", astar_params.allow_diagonal);
  get_parameter("reference_speed", optimizer_params.reference_speed);
  get_parameter("min_segment_time", optimizer_params.min_segment_time);
  get_parameter("sample_spacing", optimizer_params.sample_spacing);
  get_parameter("yaw_rate_limit", yaw_params.yaw_rate_limit);
  get_parameter("footprint_length", footprint_params.length);
  get_parameter("footprint_width", footprint_params.width);
  get_parameter("footprint_safety_margin", footprint_params.safety_margin);
  get_parameter("local_repair_enabled", repair_params.enabled);
  get_parameter("local_repair_max_iterations", repair_params.max_iterations);
  get_parameter("local_repair_search_radius", repair_params.search_radius);

  footprint_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  footprint_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;
  repair_params.obstacle_value_threshold = astar_params.obstacle_value_threshold;
  repair_params.unknown_is_obstacle = astar_params.unknown_is_obstacle;

  astar_.setParams(astar_params);
  optimizer_.setParams(optimizer_params);
  yaw_planner_.setParams(yaw_params);
  safety_checker_.setParams(footprint_params);
  collision_repair_.setParams(repair_params);
}

void MincoPlannerNode::onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_grid_ = msg;
}

void MincoPlannerNode::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (!latest_grid_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No traversability grid received yet.");
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  if (!lookupStartPose(start)) {
    RCLCPP_WARN(get_logger(), "Cannot plan because start pose lookup failed.");
    return;
  }

  geometry_msgs::msg::PoseStamped goal = *msg;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = latest_grid_->header.frame_id.empty() ? global_frame_ :
      latest_grid_->header.frame_id;
  }

  const auto astar_result = astar_.plan(*latest_grid_, start, goal);
  if (!astar_result.success) {
    RCLCPP_WARN(
      get_logger(), "A* failed: %s expanded=%d",
      astar_result.reason.c_str(), astar_result.expanded_nodes);
    return;
  }

  ReferenceTrajectory reference = optimizer_.optimize(astar_result.path);
  yaw_planner_.apply(reference, tf2::getYaw(start.pose.orientation));
  FootprintSafetyResult safety = safety_checker_.check(reference, *latest_grid_);
  if (!safety.safe && collision_repair_.repair(reference, safety, *latest_grid_)) {
    yaw_planner_.apply(reference, tf2::getYaw(start.pose.orientation));
    safety = safety_checker_.check(reference, *latest_grid_);
  }

  raw_path_pub_->publish(astar_result.path);
  reference_path_pub_->publish(toPath(reference));
  marker_pub_->publish(visualizer_.buildMarkers(astar_result.path, reference, safety));

  RCLCPP_INFO(
    get_logger(),
    "planned raw_points=%zu reference_points=%zu length=%.2f time=%.2f collisions=%zu expanded=%d",
    astar_result.path.poses.size(),
    reference.points.size(),
    reference.totalLength(),
    reference.totalTime(),
    safety.collisions.size(),
    astar_result.expanded_nodes);
}

bool MincoPlannerNode::lookupStartPose(geometry_msgs::msg::PoseStamped & start) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      latest_grid_ && !latest_grid_->header.frame_id.empty() ? latest_grid_->header.frame_id :
      global_frame_,
      robot_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    start.header = transform.header;
    start.pose.position.x = transform.transform.translation.x;
    start.pose.position.y = transform.transform.translation.y;
    start.pose.position.z = transform.transform.translation.z;
    start.pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "TF lookup %s -> %s failed: %s",
      global_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}

nav_msgs::msg::Path MincoPlannerNode::toPath(const ReferenceTrajectory & trajectory) const
{
  nav_msgs::msg::Path path;
  path.header = trajectory.header;
  path.poses.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = trajectory.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.z = std::sin(0.5 * point.yaw);
    pose.pose.orientation.w = std::cos(0.5 * point.yaw);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace minco_planner

RCLCPP_COMPONENTS_REGISTER_NODE(minco_planner::MincoPlannerNode)
