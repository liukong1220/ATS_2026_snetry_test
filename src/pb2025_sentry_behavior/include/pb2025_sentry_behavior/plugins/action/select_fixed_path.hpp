#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_FIXED_PATH_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_FIXED_PATH_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SelectFixedPathAction : public BT::SyncActionNode
{
public:
  SelectFixedPathAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SelectFixedPathAction");
  std::vector<geometry_msgs::msg::Point> goal_points_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_FIXED_PATH_HPP_
