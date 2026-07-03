#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__ADVANCE_PATROL_CURSOR_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__ADVANCE_PATROL_CURSOR_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace ats_sentry_behavior
{

class AdvancePatrolCursorAction : public BT::StatefulActionNode
{
public:
  AdvancePatrolCursorAction(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override {}
  BT::NodeStatus commitAdvance();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("AdvancePatrolCursorAction");
  rclcpp::Time release_time_{0, 0, RCL_ROS_TIME};
  double hold_duration_s_ = 0.0;
  int pending_cursor_ = 0;
  int pending_direction_ = 1;
  bool waiting_ = false;
};

}  // namespace ats_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__ADVANCE_PATROL_CURSOR_HPP_
