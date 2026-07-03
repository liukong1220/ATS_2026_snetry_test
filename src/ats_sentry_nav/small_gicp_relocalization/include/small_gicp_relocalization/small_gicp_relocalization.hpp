 

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  bool shouldRunRegistration();
  double accumulatedCloudAgeSeconds() const;
  std::optional<Eigen::Isometry3d> getCurrentRobotBaseToOdom() const;
  double translationDeltaFromLastTrigger(const Eigen::Isometry3d & current_robot_base_to_odom) const;
  double yawDeltaFromLastTrigger(const Eigen::Isometry3d & current_robot_base_to_odom) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  int num_threads_;
  int num_neighbors_;
  int min_source_points_;
  int min_inliers_;
  float global_leaf_size_;
  float registered_leaf_size_;
  float max_dist_sq_;
  double max_registration_error_;
  bool log_registration_details_;
  double registration_interval_s_;
  double max_accumulation_age_s_;
  double min_registration_translation_delta_;
  double min_registration_yaw_delta_;
  double initial_pose_force_registration_window_s_;
  double transform_future_offset_s_;
  double max_scan_stamp_lag_s_;
  std::vector<double> init_pose_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  std::optional<rclcpp::Time> first_accumulated_scan_time_;
  std::optional<rclcpp::Time> initial_pose_override_time_;
  bool has_received_scan_{false};
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  std::optional<Eigen::Isometry3d> last_registration_robot_base_to_odom_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr source_;

  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
