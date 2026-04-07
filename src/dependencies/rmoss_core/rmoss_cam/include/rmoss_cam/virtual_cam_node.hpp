 
#ifndef RMOSS_CAM__VIRTUAL_CAM_NODE_HPP_
#define RMOSS_CAM__VIRTUAL_CAM_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_cam/virtual_cam.hpp"
#include "rmoss_cam/cam_server.hpp"

namespace rmoss_cam
{
// Node wrapper for VirtualCam.
class VirtualCamNode
{
public:
  explicit VirtualCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rmoss_cam::CamInterface> cam_dev_;
  std::shared_ptr<rmoss_cam::CamServer> cam_server_;
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__VIRTUAL_CAM_NODE_HPP_
