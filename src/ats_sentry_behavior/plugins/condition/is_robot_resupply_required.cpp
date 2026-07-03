#include "ats_sentry_behavior/plugins/condition/is_robot_resupply_required.hpp"

#include <algorithm>
#include <functional>

namespace ats_sentry_behavior
{

namespace
{

constexpr char kResupplyRequiredLatchKey[] = "decision_resupply_required_latch";

}  // namespace

IsRobotResupplyRequiredCondition::IsRobotResupplyRequiredCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsRobotResupplyRequiredCondition::tickCondition, this), config),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.resource_policy.resupply_enter_hp", enter_hp_);
  node_->get_parameter("decision.resource_policy.resupply_exit_hp", exit_hp_);
  node_->get_parameter("decision.resource_policy.resupply_enter_ammo", enter_ammo_);
  node_->get_parameter("decision.resource_policy.resupply_exit_ammo", exit_ammo_);
  node_->get_parameter(
    "decision.resource_policy.require_valid_ammo_before_resupply",
    require_valid_ammo_before_resupply_);
}

BT::NodeStatus IsRobotResupplyRequiredCondition::tickCondition()
{
  auto robot_status = getInput<ats_rm_interfaces::msg::RobotStatus>("robot_status");
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
  if (ammo > enter_ammo_) {
    has_seen_valid_ammo_ = true;
  }
  const bool ammo_status_trusted =
    !require_valid_ammo_before_resupply_ || has_seen_valid_ammo_ || ammo > 0;
  const bool hp_enter_trigger = hp <= enter_hp_;
  const bool ammo_enter_trigger = ammo_status_trusted && ammo <= enter_ammo_;
  const bool hp_exit_hold = hp < exit_hp;
  const bool ammo_exit_hold = ammo_status_trusted && ammo < exit_ammo;
  const bool previous_latched = latched;

  if (latched) {
    latched = hp_exit_hold || ammo_exit_hold;
  } else {
    latched = hp_enter_trigger || ammo_enter_trigger;
  }

  if (!has_last_latched_state_ || latched != last_latched_state_) {
    RCLCPP_INFO(
      logger_,
      "[%s] resupply %s -> %s (hp=%d enter_hp=%d exit_hp=%d ammo=%d enter_ammo=%d exit_ammo=%d "
      "ammo_trusted=%d enter_reason: hp=%d ammo=%d hold_reason: hp=%d ammo=%d)",
      name().c_str(), previous_latched ? "latched" : "clear", latched ? "latched" : "clear",
      hp, enter_hp_, exit_hp, ammo, enter_ammo_, exit_ammo_,
      static_cast<int>(ammo_status_trusted),
      static_cast<int>(hp_enter_trigger), static_cast<int>(ammo_enter_trigger),
      static_cast<int>(hp_exit_hold), static_cast<int>(ammo_exit_hold));
    has_last_latched_state_ = true;
    last_latched_state_ = latched;
  } else {
    RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[%s] resupply=%s hp=%d/%d-%d ammo=%d/%d-%d ammo_trusted=%d",
      name().c_str(), latched ? "latched" : "clear",
      hp, enter_hp_, exit_hp, ammo, enter_ammo_, exit_ammo_,
      static_cast<int>(ammo_status_trusted));
  }

  root_blackboard->set(kResupplyRequiredLatchKey, latched);
  return latched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsRobotResupplyRequiredCondition::providedPorts()
{
  return {BT::InputPort<ats_rm_interfaces::msg::RobotStatus>(
    "robot_status", "{@referee_robotStatus}", "Robot status from referee")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::IsRobotResupplyRequiredCondition>(
    "IsRobotResupplyRequired");
}
