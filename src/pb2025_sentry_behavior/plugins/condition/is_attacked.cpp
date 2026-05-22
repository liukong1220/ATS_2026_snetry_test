#include "pb2025_sentry_behavior/plugins/condition/is_attacked.hpp"

#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace pb2025_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedCondition::checkIsAttacked, this), config)
{
}

BT::NodeStatus IsAttackedCondition::checkIsAttacked()
{
  const auto node = decision::getNodeFromBlackboard(*this);
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_DEBUG(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  double stop_after_s = kDefaultSpinStopAfterNoHpDropSeconds;
  getInput("stop_after_s", stop_after_s);
  // stop_after_s 是“无新掉血后的保持时间”，不是“固定自旋总时长”。
  double min_spin_duration_s = kDefaultMinimumSpinDurationSeconds;
  getInput("minimum_spin_duration_s", min_spin_duration_s);

  const bool armor_hit_reason = msg->hp_deduction_reason == msg->ARMOR_HIT;
  const bool fresh_hp_drop = msg->is_hp_deduced;
  const bool fresh_armor_event =
    armor_hit_reason &&
    (!last_is_hp_deduced_ || last_hp_deduction_reason_ != msg->hp_deduction_reason ||
    last_armor_id_ != msg->armor_id);
  // 当前策略：
  // 1. 只要本拍识别到掉血，立刻触发；
  // 2. 即便 current_hp 没再变化，只要裁判受击原因/装甲面出现新的装甲命中事件，也再次刷新锁存。
  const bool is_attacked = fresh_hp_drop || fresh_armor_event;

  if (is_attacked) {
    RCLCPP_INFO(
      logger_,
      "[%s] attacked trigger fresh_hp_drop=%d fresh_armor_event=%d hp=%u armor_id=%u reason=%u hold=%.2fs min_spin=%.2fs",
      name().c_str(), static_cast<int>(fresh_hp_drop), static_cast<int>(fresh_armor_event),
      msg->current_hp, msg->armor_id, msg->hp_deduction_reason, stop_after_s, min_spin_duration_s);
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
    last_attack_time_ = node->now();
    minimum_spin_until_ = last_attack_time_ + rclcpp::Duration::from_seconds(min_spin_duration_s);
    attack_latched_ = true;
  }

  if (attack_latched_) {
    // 只要距离最近一次掉血还没超过 stop_after_s，就持续认为“正在受击处理中”。
    const double hold_elapsed_s = (node->now() - last_attack_time_).seconds();
    const bool minimum_spin_active = node->now() <= minimum_spin_until_;
    if (hold_elapsed_s <= stop_after_s || minimum_spin_active) {
      RCLCPP_INFO_THROTTLE(
        logger_, *node->get_clock(), 500,
        "[%s] attacked latch active elapsed=%.2fs/%.2fs min_spin_active=%d yaw=%.2f",
        name().c_str(), hold_elapsed_s, stop_after_s, static_cast<int>(minimum_spin_active),
        last_attack_yaw_);
      setOutput("gimbal_pitch", 0.0F);
      setOutput("gimbal_yaw", last_attack_yaw_);
      last_is_hp_deduced_ = msg->is_hp_deduced;
      last_hp_deduction_reason_ = msg->hp_deduction_reason;
      last_armor_id_ = msg->armor_id;
      return BT::NodeStatus::SUCCESS;
    }
    // 超过持续时间仍没有新的掉血，则停止锁存。
    RCLCPP_INFO(
      logger_, "[%s] attacked latch timeout after %.2fs without refresh",
      name().c_str(), stop_after_s);
    attack_latched_ = false;
  }

  last_is_hp_deduced_ = msg->is_hp_deduced;
  last_hp_deduction_reason_ = msg->hp_deduction_reason;
  last_armor_id_ = msg->armor_id;

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
    BT::InputPort<double>(
      "minimum_spin_duration_s", "{@decision_hit_minimum_spin_duration_s}",
      "单次受击触发后最少保持自旋的时间，避免只转一拍就停止"),
    BT::OutputPort<float>(
      "gimbal_pitch", "{gimbal_pitch}",
      "输出受击时的 pitch，当前固定为 0.0"),
    BT::OutputPort<float>(
      "gimbal_yaw", "{gimbal_yaw}", "输出受击装甲方向对应的 yaw")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsAttackedCondition>("IsAttacked");
}
