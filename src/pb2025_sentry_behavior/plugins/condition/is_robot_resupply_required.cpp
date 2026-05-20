#include "pb2025_sentry_behavior/plugins/condition/is_robot_resupply_required.hpp"

#include <algorithm>
#include <functional>

namespace pb2025_sentry_behavior
{

namespace
{

constexpr char kResupplyRequiredLatchKey[] = "decision_resupply_required_latch";

}  // namespace

IsRobotResupplyRequiredCondition::IsRobotResupplyRequiredCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsRobotResupplyRequiredCondition::tickCondition, this), config)
{
  const auto node = decision::getNodeFromBlackboard(*this);
  node->get_parameter("decision.resource_policy.resupply_enter_hp", enter_hp_);
  node->get_parameter("decision.resource_policy.resupply_exit_hp", exit_hp_);
  node->get_parameter("decision.resource_policy.resupply_enter_ammo", enter_ammo_);
  node->get_parameter("decision.resource_policy.resupply_exit_ammo", exit_ammo_);
}

BT::NodeStatus IsRobotResupplyRequiredCondition::tickCondition()
{
  auto robot_status = getInput<pb_rm_interfaces::msg::RobotStatus>("robot_status");
  if (!robot_status) {
    RCLCPP_DEBUG(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto root_blackboard = config().blackboard->rootBlackboard();
  if (root_blackboard == nullptr) {
    RCLCPP_ERROR(logger_, "BehaviorTree root blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  bool latched = false;
  if (!root_blackboard->get(kResupplyRequiredLatchKey, latched)) {
    latched = false;
  }

  const int hp = static_cast<int>(robot_status->current_hp);
  const int ammo = static_cast<int>(robot_status->projectile_allowance_17mm);
  const int exit_hp = std::max(exit_hp_, enter_hp_);
  const int exit_ammo = std::max(exit_ammo_, enter_ammo_);

  if (latched) {
    latched = hp < exit_hp || ammo < exit_ammo;
  } else {
    latched = hp <= enter_hp_ || ammo <= enter_ammo_;
  }

  root_blackboard->set(kResupplyRequiredLatchKey, latched);
  return latched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsRobotResupplyRequiredCondition::providedPorts()
{
  return {BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
    "robot_status", "{@referee_robotStatus}", "Robot status from referee")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsRobotResupplyRequiredCondition>(
    "IsRobotResupplyRequired");
}
