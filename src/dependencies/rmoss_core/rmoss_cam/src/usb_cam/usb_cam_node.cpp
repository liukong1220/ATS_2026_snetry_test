 
#include "rmoss_cam/usb_cam_node.hpp"

#include <memory>
#include <string>

namespace rmoss_cam
{
UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("usb_cam", options);
  std::string usb_cam_path = "/dev/video0";
  // declare and get parameters
  node_->declare_parameter("usb_cam_path", usb_cam_path);
  node_->get_parameter("usb_cam_path", usb_cam_path);
  // create camera device
  cam_dev_ = std::make_shared<UsbCam>(usb_cam_path);
  // create task server
  cam_server_ = std::make_shared<CamServer>(node_, cam_dev_);
}

}  // namespace rmoss_cam

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmoss_cam::UsbCamNode)
