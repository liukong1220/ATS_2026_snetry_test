#include "pb2025_sentry_behavior/plugins/action/hold_stop_flag.hpp"

#include "pb2025_sentry_behavior/decision_utils.hpp"

namespace pb2025_sentry_behavior
{

HoldStopFlagAction::HoldStopFlagAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config), node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.decision_config.waypoint_stop_duration_s", hold_duration_s_);
  stop_flag_pub_ = node_->create_publisher<std_msgs::msg::Bool>("stop_flag", 10);
}

BT::NodeStatus HoldStopFlagAction::onStart()
{
  if (hold_duration_s_ <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  publishStopFlag(true);
  stop_flag_published_ = true;
  release_time_ = node_->now() + rclcpp::Duration::from_seconds(hold_duration_s_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HoldStopFlagAction::onRunning()
{
  if (hold_duration_s_ <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  if (node_->now() >= release_time_) {
    publishStopFlag(false);
    stop_flag_published_ = false;
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void HoldStopFlagAction::onHalted()
{
  if (stop_flag_published_) {
    publishStopFlag(false);
    stop_flag_published_ = false;
  }
}

void HoldStopFlagAction::publishStopFlag(bool stop)
{
  std_msgs::msg::Bool message;
  message.data = stop;
  stop_flag_pub_->publish(message);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::HoldStopFlagAction>("HoldStopFlag");
}
