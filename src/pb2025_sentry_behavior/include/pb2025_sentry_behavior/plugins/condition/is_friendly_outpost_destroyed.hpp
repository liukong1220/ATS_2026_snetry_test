#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_FRIENDLY_OUTPOST_DESTROYED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_FRIENDLY_OUTPOST_DESTROYED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsFriendlyOutpostDestroyedCondition : public BT::SimpleConditionNode
{
public:
  IsFriendlyOutpostDestroyedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsFriendlyOutpostDestroyed");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_FRIENDLY_OUTPOST_DESTROYED_HPP_
