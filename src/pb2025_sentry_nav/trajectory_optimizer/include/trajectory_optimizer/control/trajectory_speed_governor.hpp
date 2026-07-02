// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TRAJECTORY_SPEED_GOVERNOR_HPP_
#define TRAJECTORY_OPTIMIZER__TRAJECTORY_SPEED_GOVERNOR_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/trajectory_profile_msg.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace trajectory_optimizer
{

class TrajectorySpeedGovernor : public rclcpp::Node
{
public:
  explicit TrajectorySpeedGovernor(const rclcpp::NodeOptions & options);

private:
  void profileCallback(const sp_msgs::msg::TrajectoryProfileMsg::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishGovernedCmd();
  void publishProfileMarkers(const sp_msgs::msg::TrajectoryProfileMsg & msg);

  rclcpp::Subscription<sp_msgs::msg::TrajectoryProfileMsg>::SharedPtr profile_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr governed_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string profile_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string marker_topic_{"trajectory_profile_markers"};
  bool enabled_ = true;
  double min_speed_scale_ = 0.35;
  double curvature_brake_gain_ = 0.7;
  int curvature_window_points_ = 12;
  double curvature_peak_weight_ = 0.35;
  // 任务2：不仅看曲率缩放比例，还要真正服从 profile 给出的近端绝对目标速度。
  bool use_profile_target_speed_limit_ = true;
  double profile_target_speed_limit_margin_ = 0.05;
  double speed_scale_filter_gain_ = 0.25;
  double speed_scale_rise_rate_ = 4.0;
  double speed_scale_fall_rate_ = 1.8;
  double current_speed_scale_ = 1.0;
  double applied_speed_scale_ = 1.0;
  double profile_speed_cap_ = std::numeric_limits<double>::infinity();
  geometry_msgs::msg::Twist latest_cmd_vel_;
  bool has_cmd_vel_ = false;
  std::chrono::steady_clock::time_point last_publish_steady_time_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TRAJECTORY_SPEED_GOVERNOR_HPP_
