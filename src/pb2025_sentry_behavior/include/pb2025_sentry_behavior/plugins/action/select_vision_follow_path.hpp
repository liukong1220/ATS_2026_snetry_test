#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SelectVisionFollowPathAction : public BT::SyncActionNode
{
public:
  SelectVisionFollowPathAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SelectVisionFollowPathAction");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_
