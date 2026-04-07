#include "pb2025_sentry_behavior/plugins/condition/is_decision_mode.hpp"

#include <algorithm>
#include <cctype>

namespace pb2025_sentry_behavior
{

namespace
{

std::string normalizeMode(std::string mode)
{
  std::transform(
    mode.begin(), mode.end(), mode.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return mode;
}

}  // namespace

IsDecisionModeCondition::IsDecisionModeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsDecisionModeCondition::tickCondition, this), config)
{
}

BT::NodeStatus IsDecisionModeCondition::tickCondition()
{
  std::string current_mode;
  std::string expected_mode;
  if (!getInput("current_mode", current_mode) || current_mode.empty()) {
    RCLCPP_WARN(logger_, "Decision simulation mode is not available on blackboard");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("mode", expected_mode) || expected_mode.empty()) {
    RCLCPP_ERROR(logger_, "IsDecisionMode did not receive mode input");
    return BT::NodeStatus::FAILURE;
  }

  return normalizeMode(current_mode) == normalizeMode(expected_mode) ?
         BT::NodeStatus::SUCCESS :
         BT::NodeStatus::FAILURE;
}

BT::PortsList IsDecisionModeCondition::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "current_mode", "{@decision_sim_mode}", "Current decision simulation mode"),
    BT::InputPort<std::string>("mode", "patrol", "Expected decision simulation mode")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsDecisionModeCondition>(
    "IsDecisionMode");
}
