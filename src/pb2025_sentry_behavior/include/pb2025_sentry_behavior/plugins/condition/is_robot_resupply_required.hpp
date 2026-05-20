#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_RESUPPLY_REQUIRED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_RESUPPLY_REQUIRED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsRobotResupplyRequiredCondition : public BT::SimpleConditionNode
{
public:
  IsRobotResupplyRequiredCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsRobotResupplyRequired");
  int enter_hp_ = 150;
  int exit_hp_ = 400;
  int enter_ammo_ = 50;
  int exit_ammo_ = 100;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ROBOT_RESUPPLY_REQUIRED_HPP_
