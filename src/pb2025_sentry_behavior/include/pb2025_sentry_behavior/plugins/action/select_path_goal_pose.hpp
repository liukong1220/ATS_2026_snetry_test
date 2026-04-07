#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_PATH_GOAL_POSE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_PATH_GOAL_POSE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class SelectPathGoalPoseAction : public BT::SyncActionNode
{
public:
  SelectPathGoalPoseAction(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SelectPathGoalPoseAction");
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_PATH_GOAL_POSE_HPP_
