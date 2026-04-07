 

#include "pb2025_sentry_behavior/plugins/action/pub_nav2_goal.hpp"

#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

PubNav2GoalAction::PubNav2GoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubNav2GoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(logger(), "PubNav2GoalAction did not receive a valid goal input");
    return false;
  }

  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.pose = goal->pose;
  return true;
}

BT::PortsList PubNav2GoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubNav2GoalAction, "PubNav2Goal");
