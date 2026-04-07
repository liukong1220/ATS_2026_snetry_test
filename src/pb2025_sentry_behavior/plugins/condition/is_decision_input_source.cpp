#include "pb2025_sentry_behavior/plugins/condition/is_decision_input_source.hpp"

#include <algorithm>
#include <cctype>

namespace pb2025_sentry_behavior
{

namespace
{

std::string normalizeSource(std::string source)
{
  std::transform(
    source.begin(), source.end(), source.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return source;
}

}  // namespace

IsDecisionInputSourceCondition::IsDecisionInputSourceCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsDecisionInputSourceCondition::tickCondition, this), config)
{
}

BT::NodeStatus IsDecisionInputSourceCondition::tickCondition()
{
  std::string current_source;
  std::string expected_source;
  if (!getInput("current_source", current_source) || current_source.empty()) {
    RCLCPP_WARN(logger_, "Decision input source is not available on blackboard");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("source", expected_source) || expected_source.empty()) {
    RCLCPP_ERROR(logger_, "IsDecisionInputSource did not receive source input");
    return BT::NodeStatus::FAILURE;
  }

  return normalizeSource(current_source) == normalizeSource(expected_source) ?
         BT::NodeStatus::SUCCESS :
         BT::NodeStatus::FAILURE;
}

BT::PortsList IsDecisionInputSourceCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "current_source", "{@decision_input_source}", "Current decision input source"),
    BT::InputPort<std::string>("source", "referee", "Expected decision input source")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDecisionInputSourceCondition>(
    "IsDecisionInputSource");
}
