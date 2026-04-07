 

#include "pb2025_sentry_behavior/plugins/action/pub_spin_speed.hpp"

namespace pb2025_sentry_behavior
{

PublishSpinSpeedAction::PublishSpinSpeedAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishSpinSpeedAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<double>("spin_speed", 0.0, "Angular Z velocity (rad/s)"),
  });
}

bool PublishSpinSpeedAction::setMessage(example_interfaces::msg::Float32 & msg)
{
  double spin_speed = 0.0;
  getInput("spin_speed", spin_speed);

  msg.data = spin_speed;

  return true;
}

bool PublishSpinSpeedAction::setHaltMessage(example_interfaces::msg::Float32 & msg)
{
  msg.data = 0;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishSpinSpeedAction, "PublishSpinSpeed");
