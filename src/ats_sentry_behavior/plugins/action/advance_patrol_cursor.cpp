#include "ats_sentry_behavior/plugins/action/advance_patrol_cursor.hpp"

#include "ats_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ats_sentry_behavior
{

BT::NodeStatus AdvancePatrolCursorAction::onStart()
{
  if (!node_) {
    node_ = decision::getNodeFromBlackboard(*this);
    node_->get_parameter("decision.decision_config.waypoint_stop_duration_s", hold_duration_s_);
  }

  if (
    !getInput("next_cursor", pending_cursor_) ||
    !getInput("next_direction", pending_direction_))
  {
    RCLCPP_WARN(logger_, "AdvancePatrolCursor did not receive next_cursor / next_direction");
    return BT::NodeStatus::FAILURE;
  }

  if (hold_duration_s_ <= 0.0) {
    return commitAdvance();
  }

  waiting_ = true;
  release_time_ = node_->now() + rclcpp::Duration::from_seconds(hold_duration_s_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AdvancePatrolCursorAction::onRunning()
{
  if (!waiting_ || hold_duration_s_ <= 0.0) {
    return commitAdvance();
  }

  if (node_->now() < release_time_) {
    return BT::NodeStatus::RUNNING;
  }

  waiting_ = false;
  return commitAdvance();
}

BT::NodeStatus AdvancePatrolCursorAction::commitAdvance()
{
  setOutput("patrol_cursor", pending_cursor_);
  setOutput("patrol_direction", pending_direction_);
  setOutput("goal_succeeded", false);
  RCLCPP_INFO(
    logger_, "Advance patrol state to cursor=%d direction=%d",
    pending_cursor_, pending_direction_);
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList AdvancePatrolCursorAction::providedPorts()
{
  return {
    BT::InputPort<int>("next_cursor", "{decision_next_patrol_cursor}", "Next patrol cursor"),
    BT::InputPort<int>(
      "next_direction", "{decision_next_patrol_direction}", "Next patrol direction"),
    BT::OutputPort<int>("patrol_cursor", "{decision_patrol_cursor}", "Updated patrol cursor"),
    BT::OutputPort<bool>(
      "goal_succeeded", "{decision_nav_goal_succeeded}",
      "Reset the completed-path latch after advancing patrol state"),
    BT::OutputPort<int>(
      "patrol_direction", "{decision_patrol_direction}", "Updated patrol direction")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::AdvancePatrolCursorAction>(
    "AdvancePatrolCursor");
}
