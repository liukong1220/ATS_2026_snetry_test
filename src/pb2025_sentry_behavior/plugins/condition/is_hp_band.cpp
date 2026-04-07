#include "pb2025_sentry_behavior/plugins/condition/is_hp_band.hpp"

namespace pb2025_sentry_behavior
{

IsHpBandCondition::IsHpBandCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsHpBandCondition::tickCondition, this), config)
{
  const auto node = decision::getNodeFromBlackboard(*this);

  node->get_parameter("decision.time_thresholds.abundant", time_thresholds_.abundant);
  node->get_parameter("decision.time_thresholds.normal", time_thresholds_.normal);
  node->get_parameter("decision.time_thresholds.tense", time_thresholds_.tense);
  node->get_parameter("decision.time_thresholds.critical", time_thresholds_.critical);

  node->get_parameter("decision.hp_thresholds.abundant.high", hp_thresholds_.abundant.high);
  node->get_parameter("decision.hp_thresholds.abundant.medium", hp_thresholds_.abundant.medium);
  node->get_parameter("decision.hp_thresholds.abundant.low", hp_thresholds_.abundant.low);

  node->get_parameter("decision.hp_thresholds.normal.high", hp_thresholds_.normal.high);
  node->get_parameter("decision.hp_thresholds.normal.medium", hp_thresholds_.normal.medium);
  node->get_parameter("decision.hp_thresholds.normal.low", hp_thresholds_.normal.low);

  node->get_parameter("decision.hp_thresholds.tense.high", hp_thresholds_.tense.high);
  node->get_parameter("decision.hp_thresholds.tense.medium", hp_thresholds_.tense.medium);
  node->get_parameter("decision.hp_thresholds.tense.low", hp_thresholds_.tense.low);

  node->get_parameter("decision.hp_thresholds.critical.high", hp_thresholds_.critical.high);
  node->get_parameter("decision.hp_thresholds.critical.medium", hp_thresholds_.critical.medium);
  node->get_parameter("decision.hp_thresholds.critical.low", hp_thresholds_.critical.low);
}

BT::NodeStatus IsHpBandCondition::tickCondition()
{
  auto game_status = getInput<pb_rm_interfaces::msg::GameStatus>("game_status");
  auto robot_status = getInput<pb_rm_interfaces::msg::RobotStatus>("robot_status");
  if (!game_status || !robot_status) {
    RCLCPP_DEBUG(logger_, "GameStatus or RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string expected_band;
  if (!getInput("band", expected_band)) {
    RCLCPP_ERROR(logger_, "IsHpBand did not receive band input");
    return BT::NodeStatus::FAILURE;
  }

  const auto stage =
    decision::classifyGameTimeStage(game_status->stage_remain_time, time_thresholds_);
  const auto stage_thresholds = decision::selectHpThresholds(stage, hp_thresholds_);
  const auto band = decision::classifyHpBand(robot_status->current_hp, stage_thresholds);

  const bool matches =
    (expected_band == "normal" && band == decision::HpBand::kNormal) ||
    (expected_band == "low" && band == decision::HpBand::kLow) ||
    (expected_band == "critical" && band == decision::HpBand::kCritical);

  return matches ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsHpBandCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "game_status", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "robot_status", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<std::string>("band", "normal", "Expected HP band")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsHpBandCondition>("IsHpBand");
}
