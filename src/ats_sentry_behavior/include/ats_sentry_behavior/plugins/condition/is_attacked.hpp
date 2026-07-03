 

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace ats_sentry_behavior
{
/**
 * @brief 受击检测条件节点。
 *
 * 功能分两层：
 * 1. 判断当前 RobotStatus 是否表示“本次发生了掉血”
 * 2. 对最近一次受击做短时间锁存，在一段时间内持续返回 SUCCESS
 *
 * 这样行为树就可以在最近一次掉血后的短时间内持续发布自旋速度，
 * 而不是只在单帧受击消息到来时转一下就停。
 */
class IsAttackedCondition : public BT::SimpleConditionNode
{
public:
  IsAttackedCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief 定义 BT 端口。
   * @return BT::PortsList 包含受击检测所需输入和输出
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief 每次 tick 执行受击判断和锁存超时判断。
   *
   * 返回 SUCCESS 表示：
   * 1. 当前帧检测到了新的掉血，或
   * 2. 虽然当前帧没有新受击，但距离最近一次掉血仍未超过 stop_after_s
   */
  BT::NodeStatus checkIsAttacked();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsAttackedCondition");
  // 一旦发生过掉血，就在短时间内保持 true，直到超时。
  bool attack_latched_ = false;
  // 最近一次触发自旋的掉血发生时间。
  rclcpp::Time last_attack_time_{0, 0, RCL_ROS_TIME};
  // 最近一次掉血对应的朝向，用于输出云台/底盘朝向。
  float last_attack_yaw_ = 0.0F;
  // stop_after_s 没有配置时使用的默认停转超时时间。
  // 这里只是兜底默认值，推荐优先通过 YAML 参数调整。
  static constexpr double kDefaultSpinStopAfterNoHpDropSeconds = 2.0;
};
}  // namespace ats_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
