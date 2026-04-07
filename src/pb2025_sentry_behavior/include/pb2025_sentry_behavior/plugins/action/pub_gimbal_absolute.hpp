 

#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GIMBAL_ABSOLUTE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GIMBAL_ABSOLUTE_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"

namespace pb2025_sentry_behavior
{

class PublishGimbalAbsolute
: public BT::RosTopicPubStatefulActionNode<pb_rm_interfaces::msg::GimbalCmd>
{
public:
  PublishGimbalAbsolute(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

protected:
  bool setMessage(pb_rm_interfaces::msg::GimbalCmd & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GIMBAL_ABSOLUTE_HPP_
