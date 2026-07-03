 

#include "ats_sentry_behavior/plugins/condition/is_game_status.hpp"
#include "ats_sentry_behavior/decision_utils.hpp"

namespace ats_sentry_behavior
{

IsGameStatusCondition::IsGameStatusCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameStatusCondition::checkGameStart, this), config),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
}

BT::NodeStatus IsGameStatusCondition::checkGameStart()
{
  int expected_game_progress, min_remain_time, max_remain_time;
  auto msg = getInput<pb_rm_interfaces::msg::GameStatus>("key_port");
  if (!msg) {
    RCLCPP_DEBUG(logger_, "GameStatus message is not available yet");
    return BT::NodeStatus::FAILURE;
  }

  getInput("expected_game_progress", expected_game_progress);
  getInput("min_remain_time", min_remain_time);
  getInput("max_remain_time", max_remain_time);

  RCLCPP_DEBUG(
    logger_, "Checking: Progress(%d/%d), Remain Time(%ds) in [%d-%d]",
    static_cast<int>(msg->game_progress), expected_game_progress, msg->stage_remain_time,
    min_remain_time, max_remain_time);

  const bool is_progress_match = (msg->game_progress == expected_game_progress);
  const bool is_time_in_range =
    (msg->stage_remain_time >= min_remain_time) && (msg->stage_remain_time <= max_remain_time);
  const bool matched = is_progress_match && is_time_in_range;

  if (!has_last_result_ || matched != last_result_) {
    RCLCPP_INFO(
      logger_,
      "[%s] game_status %s: progress=%d expected=%d remain=%ds range=[%d,%d]",
      name().c_str(), matched ? "matched" : "not_matched",
      static_cast<int>(msg->game_progress), expected_game_progress, msg->stage_remain_time,
      min_remain_time, max_remain_time);
    has_last_result_ = true;
    last_result_ = matched;
  } else {
    RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "[%s] game_status=%s progress=%d expected=%d remain=%ds range=[%d,%d]",
      name().c_str(), matched ? "matched" : "not_matched",
      static_cast<int>(msg->game_progress), expected_game_progress, msg->stage_remain_time,
      min_remain_time, max_remain_time);
  }

  return matched ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGameStatusCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "key_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
    BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
    BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
  };
}
}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}
