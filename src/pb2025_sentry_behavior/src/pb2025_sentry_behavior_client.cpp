 

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_client.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

namespace pb2025_sentry_behavior
{

SentryBehaviorClient::SentryBehaviorClient(const rclcpp::NodeOptions & options)
: Node("sentry_behavior_client", options)
{
  declare_parameter<std::string>("target_tree", "test_attacked_feedback");
  get_parameter("target_tree", target_tree_);

  action_client_ = rclcpp_action::create_client<BTExecuteTree>(this, "pb2025_sentry_behavior");

  if (!action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(get_logger(), "Action server not available!");
    return;
  }

  timer_ = create_wall_timer(500ms, std::bind(&SentryBehaviorClient::sendGoal, this));
}

void SentryBehaviorClient::sendGoal()
{
  if (goal_sent_ || goal_active_) {
    return;
  }

  using namespace std::placeholders;
  timer_->cancel();
  goal_sent_ = true;

  auto goal_msg = BTExecuteTree::Goal();
  goal_msg.target_tree = target_tree_;

  RCLCPP_INFO(get_logger(), "Sending goal to execute Behavior Tree: %s", target_tree_.c_str());

  auto options = rclcpp_action::Client<BTExecuteTree>::SendGoalOptions();
  options.goal_response_callback =
    std::bind(&SentryBehaviorClient::goalResponseCallback, this, _1);
  options.feedback_callback = std::bind(&SentryBehaviorClient::feedbackCallback, this, _1, _2);
  options.result_callback = std::bind(&SentryBehaviorClient::resultCallback, this, _1);

  action_client_->async_send_goal(goal_msg, options);
}

void SentryBehaviorClient::scheduleRetry()
{
  goal_sent_ = false;
  goal_active_ = false;
  timer_->reset();
}

void SentryBehaviorClient::goalResponseCallback(GoalHandleBTExecuateTree::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Behavior tree goal was rejected by the server.");
    scheduleRetry();
    return;
  }
  goal_active_ = true;
}

void SentryBehaviorClient::resultCallback(
  const rclcpp_action::ClientGoalHandle<BTExecuteTree>::WrappedResult & result)
{
  goal_active_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Goal succeeded: %s", result.result->return_message.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Goal was canceled.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal failed: %s", result.result->return_message.c_str());
      break;
    case rclcpp_action::ResultCode::UNKNOWN:
      break;
  }
  scheduleRetry();
}

void SentryBehaviorClient::feedbackCallback(
  rclcpp_action::ClientGoalHandle<BTExecuteTree>::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const BTExecuteTree::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "Received feedback: %s", feedback->message.c_str());
}

}  // namespace pb2025_sentry_behavior

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pb2025_sentry_behavior::SentryBehaviorClient)
