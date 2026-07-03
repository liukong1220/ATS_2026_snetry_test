#include "ats_sentry_behavior/plugins/condition/is_attacked.hpp"

#include "ats_sentry_behavior/decision_utils.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace ats_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedCondition::checkIsAttacked, this), config)
{
}

BT::NodeStatus IsAttackedCondition::checkIsAttacked()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_DEBUG(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  double stop_after_s = kDefaultSpinStopAfterNoHpDropSeconds;
  getInput("stop_after_s", stop_after_s);
  // stop_after_s 是“无新掉血后的保持时间”，不是“固定自旋总时长”。

  // 当前策略：只要本次检测到掉血，就立即触发受击自旋。
  // 不再额外限制 hp_deduction_reason，目的是在高频掉血或裁判信息存在延迟时，
  // 仍能尽快进入自旋保护状态。
  const bool is_attacked = msg->is_hp_deduced;

  // 只有“确实掉血”且“掉血原因是装甲受击”才算有效受击。
  // const bool is_attacked = msg->is_hp_deduced && msg->hp_deduction_reason == msg->ARMOR_HIT;

  if (is_attacked) {
    RCLCPP_DEBUG(logger_, "HP deduction detected, trigger spin response");
    last_attack_yaw_ = 0.0F;
    switch (msg->armor_id) {
      // 以 0 号装甲为正前方，按逆时针推算受击方位。
      case 0:
        last_attack_yaw_ = 0.0F;
        break;
      case 1:
        last_attack_yaw_ = static_cast<float>(M_PI_2);
        break;
      case 2:
        last_attack_yaw_ = static_cast<float>(M_PI);
        break;
      case 3:
        last_attack_yaw_ = static_cast<float>(-M_PI_2);
        break;
      default:
        RCLCPP_WARN(logger_, "Invalid armor id: %d", msg->armor_id);
        break;
    }
    // 记录最近一次掉血触发时间，并打开锁存。
    // 后续只要在 stop_after_s 内没有超时，即使当前帧没有新受击消息，也继续返回 SUCCESS。
    const auto node = decision::getNodeFromBlackboard(*this);
    last_attack_time_ = node->now();
    attack_latched_ = true;
  }

  if (attack_latched_) {
    const auto node = decision::getNodeFromBlackboard(*this);
    // 只要距离最近一次掉血还没超过 stop_after_s，就持续认为“正在受击处理中”。
    if ((node->now() - last_attack_time_).seconds() <= stop_after_s) {
      setOutput("gimbal_pitch", 0.0F);
      setOutput("gimbal_yaw", last_attack_yaw_);
      return BT::NodeStatus::SUCCESS;
    }
    // 超过持续时间仍没有新的掉血，则停止锁存。
    attack_latched_ = false;
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsAttackedCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}",
      "裁判系统 RobotStatus 输入，内部使用 is_hp_deduced 与 armor_id"),
    BT::InputPort<double>(
      "stop_after_s", "{@decision_hit_spin_stop_after_no_hp_drop_s}",
      "若最近一次掉血后在该时长内没有新的掉血，则认为应停止自旋"),
    BT::OutputPort<float>(
      "gimbal_pitch", "{gimbal_pitch}",
      "输出受击时的 pitch，当前固定为 0.0"),
    BT::OutputPort<float>(
      "gimbal_yaw", "{gimbal_yaw}", "输出受击装甲方向对应的 yaw")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::IsAttackedCondition>("IsAttacked");
}
