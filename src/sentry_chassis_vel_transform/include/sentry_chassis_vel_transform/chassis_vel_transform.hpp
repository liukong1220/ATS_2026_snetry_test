#ifndef SENTRY_CHASSIS_VEL_TRANSFORM__CHASSIS_VEL_TRANSFORM_HPP_
#define SENTRY_CHASSIS_VEL_TRANSFORM__CHASSIS_VEL_TRANSFORM_HPP_

#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace sentry_chassis_vel_transform {

class ChassisVelTransform : public rclcpp::Node {
public:
  explicit ChassisVelTransform(const rclcpp::NodeOptions &options);

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::mutex yaw_mutex_;
  double latest_big_yaw_ = 0.0;
  bool has_big_yaw_ = false;

  geometry_msgs::msg::Twist last_output_;
  rclcpp::Time last_output_time_;
  bool has_last_output_ = false;
  rclcpp::Time last_yaw_update_time_;
  bool has_filtered_big_yaw_ = false;

  std::string joint_state_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string big_yaw_joint_name_;
  double linear_gain_ = 1.0;
  double angular_gain_ = 1.0;
  double max_linear_speed_ = 0.0;
  double max_linear_accel_ = 0.0;
  double max_angular_speed_ = 0.0;
  double max_angular_accel_ = 0.0;
  double max_yaw_transform_rate_ = 0.0;
  bool invert_big_yaw_ = false;
  bool invert_output_x_ = false;
  bool invert_output_y_ = false;
  bool transform_linear_with_big_yaw_ = true;
  bool pass_through_without_yaw_ = true;
};

} // namespace sentry_chassis_vel_transform

#endif // SENTRY_CHASSIS_VEL_TRANSFORM__CHASSIS_VEL_TRANSFORM_HPP_
