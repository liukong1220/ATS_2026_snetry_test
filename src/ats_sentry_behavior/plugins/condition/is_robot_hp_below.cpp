#include "ats_sentry_behavior/plugins/condition/is_robot_hp_below.hpp"

namespace ats_sentry_behavior
{

IsRobotHpBelowCondition::IsRobotHpBelowCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsRobotHpBelowCondition::tickCondition, this), config)
{
}

BT::NodeStatus IsRobotHpBelowCondition::tickCondition()
{
  auto robot_status = getInput<ats_rm_interfaces::msg::RobotStatus>("robot_status");
  if (!robot_status) {
    RCLCPP_DEBUG(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  int threshold = 300;
  if (!getInput("threshold", threshold)) {
    RCLCPP_ERROR(logger_, "IsRobotHpBelow did not receive threshold input");
    return BT::NodeStatus::FAILURE;
  }

  // 命中阈值即交给行为树切入防御相关分支。
  // 这里只负责“是否低血量”的条件判断，不负责姿态冷却和累计时长控制，
  // 后两者由 PublishRobotMode 在真正发布姿态时统一处理。
  return robot_status->current_hp <= threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsRobotHpBelowCondition::providedPorts()
{
  return {
    BT::InputPort<ats_rm_interfaces::msg::RobotStatus>(
      "robot_status", "{@referee_robotStatus}",
      "裁判系统 RobotStatus 输入，读取其中的 current_hp"),
    BT::InputPort<int>("threshold", 300, "防御姿态触发血量阈值，满足 current_hp <= threshold 即返回 SUCCESS")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::IsRobotHpBelowCondition>("IsRobotHpBelow");
}
