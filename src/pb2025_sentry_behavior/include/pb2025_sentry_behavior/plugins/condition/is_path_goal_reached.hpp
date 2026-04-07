#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_GOAL_REACHED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_GOAL_REACHED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class IsPathGoalReachedCondition : public BT::SimpleConditionNode
{
public:
  IsPathGoalReachedCondition(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tickCondition();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsPathGoalReachedCondition");
  double path_tolerance_ = 0.2;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_PATH_GOAL_REACHED_HPP_
