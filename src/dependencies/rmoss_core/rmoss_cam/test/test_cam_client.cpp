 
#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dummy_cam.hpp"
#include "rmoss_cam/cam_server.hpp"
#include "rmoss_cam/cam_client.hpp"

using namespace std::chrono_literals;

TEST(CamClient, callback)
{
  rclcpp::init(0, nullptr);
  auto cam_dev = std::make_shared<DummyCam>();
  auto node_options = rclcpp::NodeOptions();
  node_options.append_parameter_override("autostart", true);
  node_options.append_parameter_override("camera_name", "front_camera");
  node_options.append_parameter_override(
    "camera_info_url", "file://" + ament_index_cpp::get_package_share_directory(
      "rmoss_cam") + "/resource/image_cam_calibration.yaml");
  auto node = std::make_shared<rclcpp::Node>("test_cam_server", node_options);
  auto cam_server = std::make_shared<rmoss_cam::CamServer>(node, cam_dev);
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node2 = std::make_shared<rclcpp::Node>("test_cam_client");
  auto cam_client = std::make_shared<rmoss_cam::CamClient>(node2);
  executor->add_node(node);
  executor->add_node(node2);
  int num = 0;
  cam_client->set_camera_name("front_camera");
  cam_client->set_camera_callback(
    [&](const cv::Mat & /*img*/, const rclcpp::Time & /*stamp*/) {
      num++;
    });
  cam_client->connect();
  for (int i = 0; i < 10; i++) {
    executor->spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(num > 1, true);
  cam_client->disconnect();
  rclcpp::shutdown();
}
