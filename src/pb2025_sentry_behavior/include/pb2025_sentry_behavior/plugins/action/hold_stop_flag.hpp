#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__HOLD_STOP_FLAG_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__HOLD_STOP_FLAG_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace pb2025_sentry_behavior
{

class HoldStopFlagAction : public BT::StatefulActionNode
{
public:
  HoldStopFlagAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publishStopFlag(bool stop);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("HoldStopFlagAction");
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_flag_pub_;
  rclcpp::Time release_time_{0, 0, RCL_ROS_TIME};
  double hold_duration_s_ = 0.0;
  bool stop_flag_published_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__HOLD_STOP_FLAG_HPP_
