#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_GOAL_REACHED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_GOAL_REACHED_HPP_

#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class IsPathGoalReachedCondition : public BT::SimpleConditionNode
{
public:
  IsPathGoalReachedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();
  std::optional<geometry_msgs::msg::Point> extractGoalPoint(const nav_msgs::msg::Path & path) const;
  // 优先从行为树端口读取当前位姿；若端口里没有，再回退到根黑板上的
  // decision_current_pose，保持和 SendNavThroughPoses 使用同一套观测来源。
  std::optional<geometry_msgs::msg::PoseStamped> readObservedPoseFromBlackboard() const;
  // 当观测位姿缺失或超时后，直接从 TF 查询 base -> path_frame，
  // 避免 loopback/实车在 odom 短时停更时把“已离开终点”误判成“仍在终点附近”。
  std::optional<geometry_msgs::msg::PoseStamped> lookupObservedPoseFromTf(
    const std::string & target_frame) const;
  // 将当前位姿转换到 path 所在坐标系，再统一做到点判断。
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToPathFrame(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & path_frame) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsPathGoalReachedCondition");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double path_tolerance_ = 0.2;
  double pose_timeout_s_ = 0.5;
  std::optional<geometry_msgs::msg::Point> last_succeeded_goal_point_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_GOAL_REACHED_HPP_
