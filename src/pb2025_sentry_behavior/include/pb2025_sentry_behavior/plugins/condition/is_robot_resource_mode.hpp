#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_RESOURCE_MODE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_RESOURCE_MODE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

// 统一根据血量/弹量判断当前应处于哪种资源决策状态：
// 1. engage   : 允许正常巡航，也允许视觉接管。
// 2. resupply : 退出追击，回补给安全点。
// 3. defend   : 血量过低，优先执行防御/退防分支。
//
// 该节点内部自带迟滞与锁存，避免血量或弹量刚好卡在阈值附近时，
// 行为树在 attack / patrol / supply / defend 之间来回抖动。
class IsRobotResourceModeCondition : public BT::SimpleConditionNode
{
public:
  IsRobotResourceModeCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("IsRobotResourceModeCondition");
  int defend_enter_hp_ = 250;
  int defend_exit_hp_ = 300;
  int resupply_enter_hp_ = 100;
  int resupply_exit_hp_ = 400;
  int resupply_enter_ammo_ = 50;
  int resupply_exit_ammo_ = 100;
  bool assume_engage_when_status_missing_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_RESOURCE_MODE_HPP_
