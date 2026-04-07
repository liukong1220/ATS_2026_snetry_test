#include "pb2025_sentry_behavior/plugins/action/select_nearest_retreat_path.hpp"

#include <algorithm>

namespace pb2025_sentry_behavior
{

SelectNearestRetreatPathAction::SelectNearestRetreatPathAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  const auto node = decision::getNodeFromBlackboard(*this);

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<int64_t> raw_candidate_indices;

  node->get_parameter("decision.goal_points.x", xs);
  node->get_parameter("decision.goal_points.y", ys);
  node->get_parameter("decision.goal_points.z", zs);
  node->get_parameter(
    "decision.point_roles.low_hp_candidate_indices", raw_candidate_indices);

  goal_points_ = decision::buildGoalPoints(xs, ys, zs);
  candidate_indices_ = decision::toSizeIndices(
    raw_candidate_indices, goal_points_.size(),
    "decision.point_roles.low_hp_candidate_indices");
}

BT::NodeStatus SelectNearestRetreatPathAction::tick()
{
  if (candidate_indices_.empty()) {
    RCLCPP_ERROR(logger_, "Low HP candidate indices are empty");
    return BT::NodeStatus::FAILURE;
  }

  int selected_index = -1;
  getInput("selected_index", selected_index);

  std::size_t target_index = 0;
  if (selected_index >= 0) {
    const auto validated_index = decision::validateIndex(
      static_cast<std::size_t>(selected_index), goal_points_.size(), "selected_index");
    const auto existing_target = std::find(
      candidate_indices_.begin(), candidate_indices_.end(), validated_index);
    if (existing_target != candidate_indices_.end()) {
      target_index = validated_index;
    } else {
      target_index = candidate_indices_.front();
      RCLCPP_WARN(
        logger_,
        "Sticky low HP target index %zu is not in candidate list, fallback to first candidate",
        validated_index);
    }
  } else {
    auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
    if (current_pose) {
      target_index =
        decision::findNearestIndex(*current_pose, goal_points_, candidate_indices_);
    } else {
      target_index = candidate_indices_.front();
      RCLCPP_WARN(
        logger_, "Current pose is unavailable, fallback to first low HP candidate index %zu",
        target_index);
    }
  }

  nav_msgs::msg::Path path =
    decision::buildPathFromIndices(goal_points_, {target_index});

  setOutput("path", path);
  setOutput("selected_index", static_cast<int>(target_index));

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SelectNearestRetreatPathAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{decision_current_pose}", "Current navigation feedback pose"),
    BT::InputPort<int>(
      "selected_index", "{decision_low_hp_target_index}", "Sticky low HP target index"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Low HP retreat path"),
    BT::OutputPort<int>(
      "selected_index", "{decision_low_hp_target_index}", "Sticky low HP target index")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SelectNearestRetreatPathAction>(
    "SelectNearestRetreatPath");
}
