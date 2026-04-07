#include "pb2025_sentry_behavior/plugins/action/select_vision_target_path.hpp"

namespace pb2025_sentry_behavior
{

SelectVisionTargetPathAction::SelectVisionTargetPathAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  const auto node = decision::getNodeFromBlackboard(*this);

  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;

  node->get_parameter("decision.goal_points.x", xs);
  node->get_parameter("decision.goal_points.y", ys);
  node->get_parameter("decision.goal_points.z", zs);
  goal_points_ = decision::buildGoalPoints(xs, ys, zs);
}

BT::NodeStatus SelectVisionTargetPathAction::tick()
{
  int fallback_index = -1;
  if (!getInput("fallback_index", fallback_index) || fallback_index < 0) {
    RCLCPP_ERROR(logger_, "SelectVisionTargetPath did not receive a valid fallback_index");
    return BT::NodeStatus::FAILURE;
  }

  int suggested_goal_index = -1;
  getInput("suggested_goal_index", suggested_goal_index);

  // 优先使用视觉推荐的支援点；
  // 如果视觉侧暂时还没接地图语义，就退回到既有的 anchor 点。
  int selected_index = fallback_index;
  if (suggested_goal_index >= 0) {
    if (static_cast<std::size_t>(suggested_goal_index) < goal_points_.size()) {
      selected_index = suggested_goal_index;
    } else {
      RCLCPP_WARN(
        logger_,
        "Vision suggested goal index %d is out of range, fallback to %d",
        suggested_goal_index, fallback_index);
    }
  }

  const auto index = decision::validateIndex(
    static_cast<std::size_t>(selected_index), goal_points_.size(), "selected_index");
  // 这里仍然输出标准 nav_msgs/Path，
  // 这样下游不需要知道“这个路径是视觉分支还是常规决策分支选出来的”。
  auto path = decision::buildPathFromIndices(goal_points_, {index});

  setOutput("selected_index", selected_index);
  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList SelectVisionTargetPathAction::providedPorts()
{
  return {
    BT::InputPort<int>(
      "suggested_goal_index", "{vision_goal_index}", "Goal index suggested by vision"),
    BT::InputPort<int>(
      "fallback_index", "{@decision_anchor_target_index}",
      "Fallback goal index when vision does not provide a valid goal"),
    BT::OutputPort<int>(
      "selected_index", "{vision_selected_goal_index}", "Final selected vision goal index"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Vision support path")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SelectVisionTargetPathAction>(
    "SelectVisionTargetPath");
}
