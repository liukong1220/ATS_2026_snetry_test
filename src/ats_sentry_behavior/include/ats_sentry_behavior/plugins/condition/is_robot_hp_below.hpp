#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_HP_BELOW_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_HP_BELOW_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ats_sentry_behavior
{

/**
 * @brief 判断当前机器人血量是否低于阈值。
 *
 * 当前主树里它主要用于低血量防御分支：
 * 当 current_hp <= threshold 时返回 SUCCESS，
 * 行为树随后会进入 retreat / safe_point 等防御相关路径。
 */
class IsRobotHpBelowCondition : public BT::SimpleConditionNode
{
public:
  IsRobotHpBelowCondition(const std::string & name, const BT::NodeConfig & config);

  /// @brief 定义输入端口，包括 RobotStatus 和防御姿态血量阈值 threshold。
  static BT::PortsList providedPorts();

private:
  /// @brief 读取 RobotStatus.current_hp 并与阈值比较，决定是否进入低血量分支。
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsRobotHpBelowCondition");
};

}  // namespace ats_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_HP_BELOW_HPP_
