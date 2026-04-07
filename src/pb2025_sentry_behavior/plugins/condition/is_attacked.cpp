 

#include "pb2025_sentry_behavior/plugins/condition/is_attacked.hpp"

#include "pb_rm_interfaces/msg/robot_status.hpp"

namespace pb2025_sentry_behavior
{

IsAttackedCondition::IsAttackedCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsAttackedCondition::checkIsAttacked, this), config)
{
}

BT::NodeStatus IsAttackedCondition::checkIsAttacked()
{
  auto msg = getInput<pb_rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  const bool is_attacked = msg->is_hp_deduced && msg->hp_deduction_reason == msg->ARMOR_HIT;

  if (is_attacked) {
    RCLCPP_DEBUG(logger_, "Armor hit detected");
    float hit_armor_pose = 0.0;
    switch (msg->armor_id) {
      // Anticlockwise from armor 0
      case 0:
        hit_armor_pose = 0.0;
        break;
      case 1:
        hit_armor_pose = M_PI_2;
        break;
      case 2:
        hit_armor_pose = M_PI;
        break;
      case 3:
        hit_armor_pose = -M_PI_2;
        break;
      default:
        RCLCPP_WARN(logger_, "Invalid armor id: %d", msg->armor_id);
        break;
    }
    setOutput("gimbal_pitch", 0.0);
    setOutput("gimbal_yaw", hit_armor_pose);
  }

  return is_attacked ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsAttackedCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::OutputPort<float>(
      "gimbal_pitch", "{gimbal_pitch}",
      "Move gimbal_pitch (const 0.0) to the direction of the hit armor plate"),
    BT::OutputPort<float>(
      "gimbal_yaw", "{gimbal_yaw}", "Move gimbal_yaw to the direction of the hit armor plate")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsAttackedCondition>("IsAttacked");
}
