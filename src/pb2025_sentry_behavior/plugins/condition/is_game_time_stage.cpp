#include "pb2025_sentry_behavior/plugins/condition/is_game_time_stage.hpp"

namespace pb2025_sentry_behavior
{

IsGameTimeStageCondition::IsGameTimeStageCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameTimeStageCondition::tickCondition, this), config)
{
  const auto node = decision::getNodeFromBlackboard(*this);
  node->get_parameter("decision.time_thresholds.abundant", thresholds_.abundant);
  node->get_parameter("decision.time_thresholds.normal", thresholds_.normal);
  node->get_parameter("decision.time_thresholds.tense", thresholds_.tense);
  node->get_parameter("decision.time_thresholds.critical", thresholds_.critical);
}

BT::NodeStatus IsGameTimeStageCondition::tickCondition()
{
  auto game_status = getInput<pb_rm_interfaces::msg::GameStatus>("key_port");
  if (!game_status) {
    RCLCPP_DEBUG(logger_, "GameStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  std::string expected_stage;
  if (!getInput("stage", expected_stage)) {
    RCLCPP_ERROR(logger_, "IsGameTimeStage did not receive stage input");
    return BT::NodeStatus::FAILURE;
  }

  const auto stage =
    decision::classifyGameTimeStage(game_status->stage_remain_time, thresholds_);

  const bool matches =
    (expected_stage == "abundant" && stage == decision::GameTimeStage::kAbundant) ||
    (expected_stage == "normal" && stage == decision::GameTimeStage::kNormal) ||
    (expected_stage == "tense" && stage == decision::GameTimeStage::kTense) ||
    (expected_stage == "critical" && stage == decision::GameTimeStage::kCritical);

  return matches ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGameTimeStageCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::GameStatus>(
      "key_port", "{@referee_gameStatus}", "GameStatus port on blackboard"),
    BT::InputPort<std::string>("stage", "normal", "Expected time stage")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameTimeStageCondition>("IsGameTimeStage");
}
