
#ifndef ROSBAG2_COMPOSABLE_RECORDER__COMPOSABLE_RECORDER_HPP_
#define ROSBAG2_COMPOSABLE_RECORDER__COMPOSABLE_RECORDER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_transport/recorder.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rosbag2_composable_recorder
{
class ComposableRecorder : public rosbag2_transport::Recorder
{
public:
  explicit ComposableRecorder(const rclcpp::NodeOptions & options);
  ~ComposableRecorder();

private:
  // service callback function
  bool startRecording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  bool stopRecording(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // ---- variables
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  bool isRecording_{false};
  std::string bag_name_;
  std::string bag_prefix_;
};

}  // namespace rosbag2_composable_recorder
#endif  // ROSBAG2_COMPOSABLE_RECORDER__COMPOSABLE_RECORDER_HPP_
