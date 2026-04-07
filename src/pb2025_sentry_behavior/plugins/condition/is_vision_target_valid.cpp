#include "pb2025_sentry_behavior/plugins/condition/is_vision_target_valid.hpp"

#include <cmath>

namespace pb2025_sentry_behavior
{

IsVisionTargetValidCondition::IsVisionTargetValidCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsVisionTargetValidCondition::tickCondition, this), config),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.vision.timeout_s", default_timeout_s_);
}

BT::NodeStatus IsVisionTargetValidCondition::tickCondition()
{
  auto vision_target = getInput<sp_msgs::msg::VisionTargetMsg>("key_port");
  if (!vision_target) {
    RCLCPP_DEBUG(logger_, "Vision target message is not available on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  double timeout_s = default_timeout_s_;
  getInput("timeout_s", timeout_s);

  bool require_nav_hold = true;
  getInput("require_nav_hold", require_nav_hold);

  // 只有“已经稳定追踪”的目标才允许进入视觉接管分支，
  // 避免短时误检导致导航来回切换。
  if (!vision_target->tracking) {
    return BT::NodeStatus::FAILURE;
  }
  // `nav_hold` 是视觉侧给决策的明确接管信号，
  // 可以把“看到了目标”和“建议真的切换导航行为”区分开。
  if (require_nav_hold && !vision_target->nav_hold) {
    return BT::NodeStatus::FAILURE;
  }
  if (!std::isfinite(vision_target->target_yaw) || !std::isfinite(vision_target->target_pitch)) {
    return BT::NodeStatus::FAILURE;
  }

  const rclcpp::Time stamp(vision_target->timestamp);
  if (stamp.nanoseconds() <= 0) {
    return BT::NodeStatus::FAILURE;
  }

  // 行为树只消费“足够新鲜”的视觉结果，
  // 否则上一帧残留会让云台和导航继续追一个已经消失的目标。
  const auto age_s = (node_->now() - stamp).seconds();
  if (timeout_s > 0.0 && age_s > timeout_s) {
    RCLCPP_DEBUG(
      logger_, "Vision target expired: age=%.3fs timeout=%.3fs", age_s, timeout_s);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("gimbal_yaw", vision_target->target_yaw);
  setOutput("gimbal_pitch", vision_target->target_pitch);
  // 把目标编号和建议支援点也抛到黑板，
  // 后续可以继续扩展成目标优先级、地图点选择等逻辑。
  setOutput("target_id", static_cast<int>(vision_target->target_id));
  setOutput(
    "suggested_goal_index", static_cast<int>(vision_target->suggested_goal_index));
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList IsVisionTargetValidCondition::providedPorts()
{
  return {
    BT::InputPort<sp_msgs::msg::VisionTargetMsg>(
      "key_port", "{@sp_vision_target}", "Vision fusion message on blackboard"),
    BT::InputPort<double>(
      "timeout_s", "{@decision_vision_timeout_s}", "Maximum allowed message age in seconds"),
    BT::InputPort<bool>(
      "require_nav_hold", true, "Require nav_hold=true before considering the target valid"),
    BT::OutputPort<float>("gimbal_yaw", "{vision_gimbal_yaw}", "Vision gimbal yaw command"),
    BT::OutputPort<float>("gimbal_pitch", "{vision_gimbal_pitch}", "Vision gimbal pitch command"),
    BT::OutputPort<int>("target_id", "{vision_target_id}", "Current vision target id"),
    BT::OutputPort<int>(
      "suggested_goal_index", "{vision_goal_index}", "Suggested support goal index")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsVisionTargetValidCondition>(
    "IsVisionTargetValid");
}
