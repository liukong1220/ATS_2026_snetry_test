#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DECISION_MODE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DECISION_MODE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsDecisionModeCondition : public BT::SimpleConditionNode
{
public:
  IsDecisionModeCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsDecisionModeCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DECISION_MODE_HPP_
