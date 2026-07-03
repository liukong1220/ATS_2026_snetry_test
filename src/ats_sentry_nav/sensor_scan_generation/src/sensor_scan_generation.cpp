 

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");

  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);
  odometry_sub_.registerCallback(
    std::bind(&SensorScanGenerationNode::odometryHandler, this, std::placeholders::_1));

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(std::bind(
    &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
    std::placeholders::_2));
}

void SensorScanGenerationNode::odometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg)
{
  tf2::Transform tf_odom_to_lidar;
  tf2::Transform tf_lidar_to_chassis;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);
  tf_lidar_to_robot_base_ = getTransform(
    lidar_frame_, robot_base_frame_, odometry_msg->header.stamp);
  tf_lidar_to_chassis = getTransform(lidar_frame_, base_frame_, odometry_msg->header.stamp);

  tf_odom_to_robot_base_ = tf_odom_to_lidar * tf_lidar_to_robot_base_;
  tf_odom_to_chassis_ = tf_odom_to_lidar * tf_lidar_to_chassis;
  has_robot_base_pose_ = true;

  publishTransform(
    tf_odom_to_chassis_, odom_frame_, base_frame_, odometry_msg->header.stamp);
  publishTransform(
    tf_odom_to_robot_base_, odom_frame_, robot_base_frame_, odometry_msg->header.stamp);
  publishOdometry(
    tf_odom_to_robot_base_, odom_frame_, robot_base_frame_, odometry_msg->header.stamp);
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_odom_to_lidar;

  if (!has_robot_base_pose_) {
    odometryHandler(odometry_msg);
  }
  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(lidar_frame_, tf_odom_to_lidar.inverse(), *pcd_msg, out);
  pub_laser_cloud_->publish(out);
}

tf2::Transform SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_odometry_) {
    const double dt = (stamp - previous_odom_stamp_).seconds();
    if (dt > 1e-6) {
      const tf2::Vector3 delta_world =
        transform.getOrigin() - previous_odom_to_robot_base_.getOrigin();
      const tf2::Vector3 linear_velocity_world = delta_world / dt;

      const tf2::Matrix3x3 world_to_base(transform.getRotation().inverse());
      const tf2::Vector3 linear_velocity_base = world_to_base * linear_velocity_world;

      const tf2::Quaternion q_diff =
        transform.getRotation() * previous_odom_to_robot_base_.getRotation().inverse();
      tf2::Vector3 angular_axis = q_diff.getAxis();
      if (!std::isfinite(angular_axis.x()) || !std::isfinite(angular_axis.y()) ||
        !std::isfinite(angular_axis.z()))
      {
        angular_axis = tf2::Vector3(0.0, 0.0, 0.0);
      }
      const tf2::Vector3 angular_velocity_world = angular_axis * (q_diff.getAngle() / dt);
      const tf2::Vector3 angular_velocity_base = world_to_base * angular_velocity_world;

      out.twist.twist.linear.x = linear_velocity_base.x();
      out.twist.twist.linear.y = linear_velocity_base.y();
      out.twist.twist.linear.z = linear_velocity_base.z();
      out.twist.twist.angular.x = angular_velocity_base.x();
      out.twist.twist.angular.y = angular_velocity_base.y();
      out.twist.twist.angular.z = angular_velocity_base.z();
    }
  }

  previous_odom_to_robot_base_ = transform;
  previous_odom_stamp_ = stamp;
  has_previous_odometry_ = true;

  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
