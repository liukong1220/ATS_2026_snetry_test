 

#ifndef RMOSS_CAM__IMAGE_TASK_DEMO_NODE_HPP_
#define RMOSS_CAM__IMAGE_TASK_DEMO_NODE_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_cam/cam_client.hpp"

namespace rmoss_cam
{
class ImageTaskDemoNode
{
public:
  explicit ImageTaskDemoNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void init();
  void process(const cv::Mat & img, const rclcpp::Time & stamp);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<rmoss_cam::CamClient> cam_client_;
  std::string camera_name_;
  int cnt_{0};
  int info_fps_{200};
  double total_delay_{0};
  double start_time_{0};
  int process_time_ms_{0};
};

}  // namespace rmoss_cam

#endif  // RMOSS_CAM__IMAGE_TASK_DEMO_NODE_HPP_
