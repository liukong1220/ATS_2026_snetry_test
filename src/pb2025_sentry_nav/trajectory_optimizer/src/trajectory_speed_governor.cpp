// Copyright 2026

#include "trajectory_optimizer/trajectory_speed_governor.hpp"

#include <algorithm>
#include <cmath>

#include "std_msgs/msg/color_rgba.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace trajectory_optimizer
{

namespace
{

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::msg::ColorRGBA colorFromSpeedRatio(double ratio)
{
  const double clamped = std::max(0.0, std::min(1.0, ratio));
  return makeColor(
    static_cast<float>(0.85 - 0.35 * clamped),
    static_cast<float>(0.55 + 0.25 * clamped),
    static_cast<float>(0.25 + 0.15 * clamped),
    0.88f);
}

}  // namespace

TrajectorySpeedGovernor::TrajectorySpeedGovernor(const rclcpp::NodeOptions & options)
: Node("trajectory_speed_governor", options)
{
  declare_parameter<std::string>("profile_topic", "trajectory_profile");
  declare_parameter<std::string>("input_cmd_vel_topic", "cmd_vel_controller");
  declare_parameter<std::string>("output_cmd_vel_topic", "cmd_vel_controller_governed");
  declare_parameter<std::string>("marker_topic", marker_topic_);
  declare_parameter<double>("min_speed_scale", min_speed_scale_);
  declare_parameter<double>("curvature_brake_gain", curvature_brake_gain_);
  declare_parameter<int>("curvature_window_points", curvature_window_points_);
  declare_parameter<double>("speed_scale_filter_gain", speed_scale_filter_gain_);

  get_parameter("profile_topic", profile_topic_);
  get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  get_parameter("marker_topic", marker_topic_);
  get_parameter("min_speed_scale", min_speed_scale_);
  get_parameter("curvature_brake_gain", curvature_brake_gain_);
  get_parameter("curvature_window_points", curvature_window_points_);
  get_parameter("speed_scale_filter_gain", speed_scale_filter_gain_);

  profile_sub_ = create_subscription<sp_msgs::msg::TrajectoryProfileMsg>(
    profile_topic_, 10,
    std::bind(&TrajectorySpeedGovernor::profileCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&TrajectorySpeedGovernor::cmdVelCallback, this, std::placeholders::_1));
  governed_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
  marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&TrajectorySpeedGovernor::publishGovernedCmd, this));
}

void TrajectorySpeedGovernor::profileCallback(
  const sp_msgs::msg::TrajectoryProfileMsg::SharedPtr msg)
{
  if (msg->points.empty()) {
    current_speed_scale_ = 1.0;
    return;
  }

  const std::size_t window_points = std::max(
    std::size_t(1),
    std::min(
      msg->points.size(),
      static_cast<std::size_t>(std::max(1, curvature_window_points_))));

  double window_max_abs_curvature = 0.0;
  for (std::size_t i = 0; i < window_points; ++i) {
    window_max_abs_curvature = std::max(
      window_max_abs_curvature, std::abs(msg->points[i].curvature));
  }

  const double curvature_scale =
    1.0 / (1.0 + curvature_brake_gain_ * window_max_abs_curvature);
  const double target_speed_scale = std::max(
    min_speed_scale_, std::min(1.0, curvature_scale));
  const double filter_gain = std::max(0.0, std::min(1.0, speed_scale_filter_gain_));
  current_speed_scale_ += (target_speed_scale - current_speed_scale_) * filter_gain;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "Speed governor scale: target=%.3f filtered=%.3f kappa_window_max=%.3f window_points=%zu",
    target_speed_scale,
    current_speed_scale_,
    window_max_abs_curvature,
    window_points);

  publishProfileMarkers(*msg);
}

void TrajectorySpeedGovernor::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  has_cmd_vel_ = true;
}

void TrajectorySpeedGovernor::publishGovernedCmd()
{
  if (!has_cmd_vel_) {
    return;
  }

  geometry_msgs::msg::Twist governed = latest_cmd_vel_;
  governed.linear.x *= current_speed_scale_;
  governed.linear.y *= current_speed_scale_;
  governed.angular.z *= std::sqrt(current_speed_scale_);
  governed_cmd_pub_->publish(governed);
}

void TrajectorySpeedGovernor::publishProfileMarkers(
  const sp_msgs::msg::TrajectoryProfileMsg & msg)
{
  if (!marker_pub_ || msg.points.empty()) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker line;
  line.header = msg.header;
  line.ns = "trajectory_profile";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.022;
  line.color = makeColor(0.28f, 0.72f, 0.78f, 0.86f);

  visualization_msgs::msg::Marker points;
  points.header = msg.header;
  points.ns = "trajectory_profile";
  points.id = 1;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.055;
  points.scale.y = 0.055;
  points.scale.z = 0.055;

  visualization_msgs::msg::Marker text;
  text.header = msg.header;
  text.ns = "trajectory_profile";
  text.id = 2;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.scale.z = 0.15;
  text.color = makeColor(0.92f, 0.92f, 0.88f, 0.90f);
  text.pose.position = msg.points.back().point;
  text.pose.position.z += 0.35;
  text.text =
    "kappa_max=" + std::to_string(msg.max_abs_curvature).substr(0, 4) +
    " cost=" + std::to_string(msg.total_cost).substr(0, 6);

  for (const auto & sample : msg.points) {
    line.points.push_back(sample.point);
    points.points.push_back(sample.point);
    const double denom = std::max(1e-3, sample.speed_limit);
    points.colors.push_back(colorFromSpeedRatio(sample.speed / denom));
  }

  markers.markers.push_back(line);
  markers.markers.push_back(points);
  markers.markers.push_back(text);
  marker_pub_->publish(markers);
}

}  // namespace trajectory_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_optimizer::TrajectorySpeedGovernor)
