 
#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "dummy_cam.hpp"
#include "rmoss_cam/cam_server.hpp"

using namespace std::chrono_literals;

TEST(CamServer, reopen)
{
  rclcpp::init(0, nullptr);
  auto cam_dev = std::make_shared<DummyCam>();
  auto node_options = rclcpp::NodeOptions();
  node_options.append_parameter_override("autostart", true);
  node_options.append_parameter_override(
    "camera_info_url", "file://" + ament_index_cpp::get_package_share_directory(
      "rmoss_cam") + "/resource/image_cam_calibration.yaml");
  node_options.append_parameter_override("camera_name", "front_camera");
  auto node = std::make_shared<rclcpp::Node>("test_cam", node_options);
  auto cam_server = std::make_shared<rmoss_cam::CamServer>(node, cam_dev);
  auto spin_thread = std::thread([&]() {rclcpp::spin(node);});
  std::this_thread::sleep_for(500ms);
  cam_dev->set_falut();
  std::this_thread::sleep_for(1000ms);
  EXPECT_EQ(cam_dev->is_open(), true);
  rclcpp::shutdown();
  spin_thread.join();
  SUCCEED();
}

TEST(CamServer, camera_transport)
{
  rclcpp::init(0, nullptr);
  auto cam_dev = std::make_shared<DummyCam>();
  auto node_options = rclcpp::NodeOptions();
  node_options.append_parameter_override("autostart", true);
  node_options.append_parameter_override(
    "camera_info_url", "file://" + ament_index_cpp::get_package_share_directory(
      "rmoss_cam") + "/resource/image_cam_calibration.yaml");
  node_options.append_parameter_override("camera_name", "front_camera");
  node_options.append_parameter_override("use_image_transport_camera_publisher", true);
  auto node = std::make_shared<rclcpp::Node>("test_cam", node_options);
  auto cam_server = std::make_shared<rmoss_cam::CamServer>(node, cam_dev);
  auto spin_thread = std::thread([&]() {rclcpp::spin(node);});
  std::this_thread::sleep_for(1000ms);
  rclcpp::shutdown();
  spin_thread.join();
  SUCCEED();
}
