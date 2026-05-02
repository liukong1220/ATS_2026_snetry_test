#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_VISION_TARGET_VALID_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_VISION_TARGET_VALID_HPP_

#include <optional>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/vision_target_msg.hpp"

namespace pb2025_sentry_behavior
{

// 判断视觉融合消息是否足够新鲜、是否允许接管导航，
// 同时把云台 yaw/pitch 和目标编号提取出来交给行为树后续节点使用。
class IsVisionTargetValidCondition : public BT::SimpleConditionNode
{
public:
  IsVisionTargetValidCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();
  void resetVisionLatchState();
  BT::NodeStatus keepLatchedTargetIfAllowed(const char * reason);
  BT::NodeStatus failWithReason(const char * reason);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsVisionTargetValidCondition");
  double default_timeout_s_ = 0.5;
  double override_hold_s_ = 0.0;
  double activation_hold_s_ = 0.0;
  double switch_target_hold_s_ = 0.0;
  std::optional<rclcpp::Time> hold_until_;
  std::optional<rclcpp::Time> activation_started_at_;
  std::optional<rclcpp::Time> pending_switch_started_at_;
  float last_gimbal_yaw_ = 0.0F;
  float last_gimbal_pitch_ = 0.0F;
  int last_target_id_ = -1;
  int pending_target_id_ = -1;
  bool has_latched_target_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_VISION_TARGET_VALID_HPP_
