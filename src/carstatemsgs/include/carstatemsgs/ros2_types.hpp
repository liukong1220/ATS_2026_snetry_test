#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace geometry_msgs
{
using Point = msg::Point;
using PointStamped = msg::PointStamped;
using Pose = msg::Pose;
using PoseStamped = msg::PoseStamped;
using Quaternion = msg::Quaternion;
using TransformStamped = msg::TransformStamped;
using Transform = msg::Transform;
using Vector3 = msg::Vector3;
}  // namespace geometry_msgs

namespace nav_msgs
{
using Odometry = msg::Odometry;
using Path = msg::Path;
}  // namespace nav_msgs

namespace sensor_msgs
{
using PointCloud2 = msg::PointCloud2;
}  // namespace sensor_msgs

namespace std_msgs
{
using Bool = msg::Bool;
using ColorRGBA = msg::ColorRGBA;
using Header = msg::Header;
}  // namespace std_msgs

namespace visualization_msgs
{
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
}  // namespace visualization_msgs

namespace tf
{
using Quaternion = tf2::Quaternion;

inline Quaternion createQuaternionFromYaw(double yaw)
{
  Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return q;
}

inline Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw)
{
  Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return q;
}

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
  return tf2::toMsg(createQuaternionFromYaw(yaw));
}

inline double getYaw(const geometry_msgs::msg::Quaternion &q)
{
  return tf2::getYaw(q);
}

inline double getYaw(const geometry_msgs::msg::Transform &t)
{
  return tf2::getYaw(t.rotation);
}
}  // namespace tf

#ifndef ROS_INFO
#define ROS_INFO(...) RCLCPP_INFO(::rclcpp::get_logger("ddr_opt"), __VA_ARGS__)
#endif
#ifndef ROS_WARN
#define ROS_WARN(...) RCLCPP_WARN(::rclcpp::get_logger("ddr_opt"), __VA_ARGS__)
#endif
#ifndef ROS_ERROR
#define ROS_ERROR(...) RCLCPP_ERROR(::rclcpp::get_logger("ddr_opt"), __VA_ARGS__)
#endif
#ifndef ROS_INFO_STREAM
#define ROS_INFO_STREAM(msg) \
  do { \
    std::ostringstream _ros2_types_oss; \
    _ros2_types_oss << msg; \
    RCLCPP_INFO(::rclcpp::get_logger("ddr_opt"), "%s", _ros2_types_oss.str().c_str()); \
  } while (0)
#endif
#ifndef ROS_WARN_STREAM
#define ROS_WARN_STREAM(msg) \
  do { \
    std::ostringstream _ros2_types_oss; \
    _ros2_types_oss << msg; \
    RCLCPP_WARN(::rclcpp::get_logger("ddr_opt"), "%s", _ros2_types_oss.str().c_str()); \
  } while (0)
#endif
#ifndef ROS_ERROR_STREAM
#define ROS_ERROR_STREAM(msg) \
  do { \
    std::ostringstream _ros2_types_oss; \
    _ros2_types_oss << msg; \
    RCLCPP_ERROR(::rclcpp::get_logger("ddr_opt"), "%s", _ros2_types_oss.str().c_str()); \
  } while (0)
#endif
