 

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_SPEED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_SPEED_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace pb2025_sentry_behavior
{

class PublishSpinSpeedAction
: public BT::RosTopicPubStatefulActionNode<example_interfaces::msg::Float32>
{
public:
  PublishSpinSpeedAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(example_interfaces::msg::Float32 & msg) override;

  bool setHaltMessage(example_interfaces::msg::Float32 & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SPIN_SPEED_HPP_
