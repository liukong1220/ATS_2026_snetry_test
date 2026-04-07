 

#ifndef PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_CLIENT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_CLIENT_HPP_

#include <memory>
#include <string>

#include "btcpp_ros2_interfaces/action/execute_tree.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client.hpp"

namespace pb2025_sentry_behavior
{

class SentryBehaviorClient : public rclcpp::Node
{
public:
  using BTExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
  using GoalHandleBTExecuateTree = rclcpp_action::ClientGoalHandle<BTExecuteTree>;

  explicit SentryBehaviorClient(const rclcpp::NodeOptions & options);

private:
  void sendGoal();
  void scheduleRetry();
  void goalResponseCallback(GoalHandleBTExecuateTree::SharedPtr goal_handle);
  void resultCallback(const GoalHandleBTExecuateTree::WrappedResult & result);
  void feedbackCallback(
    GoalHandleBTExecuateTree::SharedPtr goal_handle,
    const std::shared_ptr<const BTExecuteTree::Feedback> feedback);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<BTExecuteTree>::SharedPtr action_client_;
  std::string target_tree_;
  bool goal_sent_ = false;
  bool goal_active_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PB2025_SENTRY_BEHAVIOR_CLIENT_HPP_
