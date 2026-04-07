#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_TARGET_PATH_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_TARGET_PATH_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

// 根据视觉建议导航点生成路径。
// 如果视觉没有给出合法编号，就自动回退到配置里的 anchor 点，
// 这样第一代融合系统即使视觉只给 yaw/pitch，也能先跑起来。
class SelectVisionTargetPathAction : public BT::SyncActionNode
{
public:
  SelectVisionTargetPathAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  rclcpp::Logger logger_ = rclcpp::get_logger("SelectVisionTargetPathAction");
  std::vector<geometry_msgs::msg::Point> goal_points_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_TARGET_PATH_HPP_
