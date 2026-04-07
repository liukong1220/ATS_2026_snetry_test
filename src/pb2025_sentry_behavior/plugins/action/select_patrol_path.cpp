#include "pb2025_sentry_behavior/plugins/action/select_patrol_path.hpp"

#include <algorithm>

namespace pb2025_sentry_behavior
{

SelectPatrolPathAction::SelectPatrolPathAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = decision::getNodeFromBlackboard(*this);

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  std::vector<int64_t> raw_patrol_indices;

  node_->get_parameter("decision.goal_points.x", xs);
  node_->get_parameter("decision.goal_points.y", ys);
  node_->get_parameter("decision.goal_points.z", zs);
  node_->get_parameter("decision.point_roles.patrol_indices", raw_patrol_indices);
  node_->get_parameter(
    "decision.decision_config.patrol_preview_points", patrol_preview_points_);

  goal_points_ = decision::buildGoalPoints(xs, ys, zs);
  patrol_indices_ = decision::toSizeIndices(
    raw_patrol_indices, goal_points_.size(), "decision.point_roles.patrol_indices");
}

BT::NodeStatus SelectPatrolPathAction::tick()
{
  node_->get_parameter(
    "decision.decision_config.patrol_preview_points", patrol_preview_points_);

  if (patrol_indices_.empty()) {
    RCLCPP_ERROR(logger_, "Patrol indices are empty");
    return BT::NodeStatus::FAILURE;
  }

  int patrol_cursor = 0;
  int patrol_direction = 1;
  getInput("patrol_cursor", patrol_cursor);
  getInput("patrol_direction", patrol_direction);

  if (patrol_cursor < 0 || patrol_cursor >= static_cast<int>(patrol_indices_.size())) {
    patrol_cursor = 0;
  }
  if (patrol_direction == 0) {
    patrol_direction = 1;
  }

  if (patrol_indices_.size() == 1) {
    nav_msgs::msg::Path path =
      decision::buildPathFromIndices(goal_points_, {patrol_indices_.front()});
    setOutput("path", path);
    setOutput("next_cursor", 0);
    setOutput("next_direction", 1);
    return BT::NodeStatus::SUCCESS;
  }

  const int preview_points = std::max(2, patrol_preview_points_);
  std::vector<std::size_t> route_indices;
  route_indices.reserve(static_cast<std::size_t>(preview_points));
  route_indices.push_back(patrol_indices_[static_cast<std::size_t>(patrol_cursor)]);

  auto preview_cursor = patrol_cursor;
  auto preview_direction = patrol_direction;
  for (int step = 1; step < preview_points; ++step) {
    const auto [next_cursor, next_direction] =
      decision::computeNextPatrolState(
        preview_cursor, preview_direction, patrol_indices_.size());
    route_indices.push_back(patrol_indices_[static_cast<std::size_t>(next_cursor)]);
    preview_cursor = next_cursor;
    preview_direction = next_direction;
  }

  nav_msgs::msg::Path path =
    decision::buildPathFromIndices(goal_points_, route_indices);

  setOutput("path", path);
  setOutput("next_cursor", preview_cursor);
  setOutput("next_direction", preview_direction);

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SelectPatrolPathAction::providedPorts()
{
  return {
    BT::InputPort<int>("patrol_cursor", "{decision_patrol_cursor}", "Current patrol cursor"),
    BT::InputPort<int>(
      "patrol_direction", "{decision_patrol_direction}", "Current patrol direction"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Patrol path"),
    BT::OutputPort<int>("next_cursor", "{decision_next_patrol_cursor}", "Next patrol cursor"),
    BT::OutputPort<int>(
      "next_direction", "{decision_next_patrol_direction}", "Next patrol direction")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SelectPatrolPathAction>("SelectPatrolPath");
}
