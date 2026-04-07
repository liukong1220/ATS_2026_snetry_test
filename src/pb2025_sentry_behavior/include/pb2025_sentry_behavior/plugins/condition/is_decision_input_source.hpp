#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DECISION_INPUT_SOURCE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DECISION_INPUT_SOURCE_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsDecisionInputSourceCondition : public BT::SimpleConditionNode
{
public:
  IsDecisionInputSourceCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsDecisionInputSourceCondition");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DECISION_INPUT_SOURCE_HPP_
