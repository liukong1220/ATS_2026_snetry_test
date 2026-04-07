#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "rosbag2_composable_recorder/composable_recorder.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<rosbag2_composable_recorder::ComposableRecorder>(rclcpp::NodeOptions());

  RCLCPP_INFO(node->get_logger(), "recorder node started up!");
  rclcpp::spin(node);  // should not return
  rclcpp::shutdown();
  return 0;
}
