 

#include "pb2025_sentry_behavior/plugins/condition/is_status_ok.hpp"

namespace pb2025_sentry_behavior
{

IsStatusOKCondition::IsStatusOKCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKCondition::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusOKCondition::checkRobotStatus()
{
  int hp_min, heat_max, ammo_min;
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_DEBUG(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("hp_min", hp_min);
  getInput("heat_max", heat_max);
  getInput("ammo_min", ammo_min);

  const bool is_hp_ok = (msg->current_hp >= hp_min);
  const bool is_heat_ok = (msg->shooter_17mm_1_barrel_heat <= heat_max);
  const bool is_ammo_ok = (msg->projectile_allowance_17mm >= ammo_min);

  return (is_hp_ok && is_heat_ok && is_ammo_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>("hp_min", 300, "Minimum HP. NOTE: Sentry init/max HP is 400"),
    BT::InputPort<int>("heat_max", 350, "Maximum heat. NOTE: Sentry heat limit is 400"),
    BT::InputPort<int>("ammo_min", 0, "Lower then minimum ammo will return FAILURE")};
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsStatusOKCondition>("IsStatusOK");
}
