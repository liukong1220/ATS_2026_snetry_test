#include "pb2025_sentry_behavior/plugins/condition/is_path_goal_reached.hpp"

namespace pb2025_sentry_behavior
{

IsPathGoalReachedCondition::IsPathGoalReachedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPathGoalReachedCondition::tickCondition, this), config)
{
  const auto node = decision::getNodeFromBlackboard(*this);
  node->get_parameter("decision.decision_config.path_tolerance", path_tolerance_);
}

BT::NodeStatus IsPathGoalReachedCondition::tickCondition()
{
  bool goal_succeeded = false;
  if (getInput("goal_succeeded", goal_succeeded) && goal_succeeded) {
    return BT::NodeStatus::SUCCESS;
  }

  auto path = getInput<nav_msgs::msg::Path>("path");
  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if (!path || !current_pose || path->poses.empty()) {
    RCLCPP_DEBUG(logger_, "Path or current pose is not available");
    return BT::NodeStatus::FAILURE;
  }

  return decision::isPathGoalReached(*current_pose, *path, path_tolerance_) ?
           BT::NodeStatus::SUCCESS :
           BT::NodeStatus::FAILURE;
}

BT::PortsList IsPathGoalReachedCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Current decision path"),
    BT::InputPort<bool>("goal_succeeded", "Whether NavigateThroughPoses already reported success for this path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{decision_current_pose}", "Current navigation feedback pose")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPathGoalReachedCondition>(
    "IsPathGoalReached");
}
