 

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <chrono>
#include <memory>

namespace BT
{

struct RosNodeParams
{
  std::weak_ptr<rclcpp::Node> nh;

  // This has different meaning based on the context:
  //
  // - RosActionNode: name of the action server
  // - RosServiceNode: name of the service
  // - RosTopicPubNode: name of the topic to publish to
  // - RosTopicSubNode: name of the topic to subscribe to
  std::string default_port_value;

  // parameters used only by service client and action clients

  // timeout when sending a request
  std::chrono::milliseconds server_timeout = std::chrono::milliseconds(1000);
  // timeout used when detecting the server the first time
  std::chrono::milliseconds wait_for_server_timeout = std::chrono::milliseconds(500);

  RosNodeParams() = default;
  RosNodeParams(std::shared_ptr<rclcpp::Node> node) : nh(node)
  {}
  RosNodeParams(std::shared_ptr<rclcpp::Node> node, const std::string& port_name)
    : nh(node), default_port_value(port_name)
  {}
};

}  // namespace BT
