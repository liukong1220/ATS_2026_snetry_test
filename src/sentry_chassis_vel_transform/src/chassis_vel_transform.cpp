#include "sentry_chassis_vel_transform/chassis_vel_transform.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

namespace sentry_chassis_vel_transform
{

ChassisVelTransform::ChassisVelTransform(const rclcpp::NodeOptions & options)
: Node("chassis_vel_transform", options)
{
  joint_state_topic_ = declare_parameter<std::string>(
    "joint_state_topic", "serial/gimbal_joint_state");
  input_cmd_vel_topic_ = declare_parameter<std::string>(
    "input_cmd_vel_topic", "cmd_vel_gimbal_yaw_odom");
  output_cmd_vel_topic_ = declare_parameter<std::string>("output_cmd_vel_topic", "/cmd_vel");
  big_yaw_joint_name_ = declare_parameter<std::string>(
    "big_yaw_joint_name", "gimbal_yaw_odom_joint");
  linear_gain_ = declare_parameter<double>("linear_gain", 1.0);
  angular_gain_ = declare_parameter<double>("angular_gain", 1.0);
  max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.0);
  max_linear_accel_ = declare_parameter<double>("max_linear_accel", 0.0);
  invert_big_yaw_ = declare_parameter<bool>("invert_big_yaw", false);
  invert_output_x_ = declare_parameter<bool>("invert_output_x", false);
  invert_output_y_ = declare_parameter<bool>("invert_output_y", false);
  pass_through_without_yaw_ = declare_parameter<bool>("pass_through_without_yaw", true);

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 10);
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, 10,
    std::bind(&ChassisVelTransform::jointStateCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&ChassisVelTransform::cmdVelCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Transforming %s from %s using %s/%s into %s",
    input_cmd_vel_topic_.c_str(), "gimbal_yaw_odom", joint_state_topic_.c_str(),
    big_yaw_joint_name_.c_str(), output_cmd_vel_topic_.c_str());
}

void ChassisVelTransform::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  const auto count = std::min(msg->name.size(), msg->position.size());
  for (std::size_t i = 0; i < count; ++i) {
    if (msg->name[i] != big_yaw_joint_name_) {
      continue;
    }

    std::lock_guard<std::mutex> lock(yaw_mutex_);
    latest_big_yaw_ = invert_big_yaw_ ? -msg->position[i] : msg->position[i];
    has_big_yaw_ = true;
    return;
  }
}

void ChassisVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double big_yaw = 0.0;
  bool has_big_yaw = false;
  {
    std::lock_guard<std::mutex> lock(yaw_mutex_);
    big_yaw = latest_big_yaw_;
    has_big_yaw = has_big_yaw_;
  }

  if (!has_big_yaw && !pass_through_without_yaw_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Dropping cmd_vel because no %s sample has been received from %s",
      big_yaw_joint_name_.c_str(), joint_state_topic_.c_str());
    return;
  }

  if (!has_big_yaw) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "No %s sample has been received from %s; publishing cmd_vel without chassis transform",
      big_yaw_joint_name_.c_str(), joint_state_topic_.c_str());
  }

  const double cos_yaw = std::cos(big_yaw);
  const double sin_yaw = std::sin(big_yaw);

  geometry_msgs::msg::Twist output = *msg;
  output.linear.x = (msg->linear.x * cos_yaw - msg->linear.y * sin_yaw) * linear_gain_;
  output.linear.y = (msg->linear.x * sin_yaw + msg->linear.y * cos_yaw) * linear_gain_;
  output.angular.z = msg->angular.z * angular_gain_;
  if (invert_output_x_) {
    output.linear.x = -output.linear.x;
  }
  if (invert_output_y_) {
    output.linear.y = -output.linear.y;
  }

  const double linear_speed = std::hypot(output.linear.x, output.linear.y);
  if (max_linear_speed_ > 0.0 && linear_speed > max_linear_speed_) {
    const double speed_scale = max_linear_speed_ / linear_speed;
    output.linear.x *= speed_scale;
    output.linear.y *= speed_scale;
  }

  const auto now = get_clock()->now();
  if (has_last_output_ && max_linear_accel_ > 0.0) {
    const double dt = (now - last_output_time_).seconds();
    if (dt > 1e-6) {
      const double max_delta = max_linear_accel_ * dt;
      const double delta_x = output.linear.x - last_output_.linear.x;
      const double delta_y = output.linear.y - last_output_.linear.y;
      const double delta_norm = std::hypot(delta_x, delta_y);
      if (delta_norm > max_delta) {
        const double delta_scale = max_delta / delta_norm;
        output.linear.x = last_output_.linear.x + delta_x * delta_scale;
        output.linear.y = last_output_.linear.y + delta_y * delta_scale;
      }
    }
  }

  last_output_ = output;
  last_output_time_ = now;
  has_last_output_ = true;

  cmd_vel_pub_->publish(output);
}

}  // namespace sentry_chassis_vel_transform

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_chassis_vel_transform::ChassisVelTransform)
