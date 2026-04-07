#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV_THROUGH_POSES_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV_THROUGH_POSES_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace pb2025_sentry_behavior
{

class SendNavThroughPosesAction : public BT::SyncActionNode
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  SendNavThroughPosesAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void resultCallback(std::uint64_t request_id, const GoalHandle::WrappedResult & result);
  void feedbackCallback(
    std::uint64_t request_id, GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void goalResponseCallback(std::uint64_t request_id, const GoalHandle::SharedPtr & goal_handle);
  void cancelCurrentGoal();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SendNavThroughPosesAction");
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  std::string action_name_;
  std::mutex mutex_;
  GoalHandle::SharedPtr current_goal_handle_;
  nav_msgs::msg::Path active_path_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_current_pose_ = false;
  bool goal_pending_ = false;
  bool last_goal_succeeded_ = false;
  std::uint64_t goal_request_id_ = 0;
  double path_compare_tolerance_ = 0.1;
  double action_server_wait_timeout_s_ = 2.0;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV_THROUGH_POSES_HPP_
