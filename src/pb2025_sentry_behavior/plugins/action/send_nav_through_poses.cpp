#include "pb2025_sentry_behavior/plugins/action/send_nav_through_poses.hpp"

namespace pb2025_sentry_behavior
{

SendNavThroughPosesAction::SendNavThroughPosesAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config), node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  action_name_ = "/navigate_through_poses";
  node_->get_parameter("decision.decision_config.nav2_action_server", action_name_);
  node_->get_parameter(
    "decision.decision_config.goal_position_tolerance", path_compare_tolerance_);
  node_->get_parameter(
    "decision.decision_config.action_server_wait_timeout_s", action_server_wait_timeout_s_);

  action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, action_name_);
}

BT::PortsList SendNavThroughPosesAction::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Decision path"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{decision_current_pose}", "Current navigation feedback pose"),
    BT::OutputPort<bool>(
      "goal_succeeded", "{decision_nav_goal_succeeded}",
      "Whether the current decision path already finished successfully")};
}

BT::NodeStatus SendNavThroughPosesAction::tick()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  if (!path || path->poses.empty()) {
    RCLCPP_ERROR(logger_, "SendNavThroughPosesAction did not receive a valid path input");
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(action_server_wait_timeout_s_))))
  {
    RCLCPP_ERROR(logger_, "Action server %s is not available", action_name_.c_str());
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_current_pose_) {
      setOutput("current_pose", latest_pose_);
    }

    const bool same_path =
      decision::pathEquivalent(active_path_, *path, path_compare_tolerance_);
    setOutput(
      "goal_succeeded",
      same_path && last_goal_succeeded_ && !goal_pending_ && !current_goal_handle_);

    if (same_path && last_goal_succeeded_ && !goal_pending_ && !current_goal_handle_) {
      return BT::NodeStatus::SUCCESS;
    }

    if ((goal_pending_ || current_goal_handle_) && same_path)
    {
      if (has_current_pose_) {
        setOutput("current_pose", latest_pose_);
      }
      return BT::NodeStatus::SUCCESS;
    }
  }

  cancelCurrentGoal();

  NavigateThroughPoses::Goal goal;
  goal.poses = path->poses;
  for (auto & pose : goal.poses) {
    pose.header.stamp = node_->now();
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = "map";
    }
  }

  std::uint64_t request_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request_id = ++goal_request_id_;
    goal_pending_ = true;
    last_goal_succeeded_ = false;
    active_path_ = *path;
  }
  setOutput("goal_succeeded", false);

  auto send_goal_options =
    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this, request_id](const GoalHandle::SharedPtr handle) {
      goalResponseCallback(request_id, handle);
    };
  send_goal_options.feedback_callback =
    [this, request_id](
      GoalHandle::SharedPtr handle,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        feedbackCallback(request_id, handle, feedback);
      };
  send_goal_options.result_callback = [this, request_id](const GoalHandle::WrappedResult & result) {
      resultCallback(request_id, result);
    };

  action_client_->async_send_goal(goal, send_goal_options);
  return BT::NodeStatus::SUCCESS;
}

void SendNavThroughPosesAction::goalResponseCallback(
  std::uint64_t request_id, const GoalHandle::SharedPtr & goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_id != goal_request_id_) {
    return;
  }
  goal_pending_ = false;
  current_goal_handle_ = goal_handle;
  if (!goal_handle) {
    last_goal_succeeded_ = false;
    RCLCPP_ERROR(logger_, "NavigateThroughPoses goal was rejected by server");
  }
}

void SendNavThroughPosesAction::feedbackCallback(
  std::uint64_t request_id, GoalHandle::SharedPtr,
  const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_id != goal_request_id_) {
    return;
  }
  latest_pose_ = feedback->current_pose;
  has_current_pose_ = true;
}

void SendNavThroughPosesAction::resultCallback(
  std::uint64_t request_id, const GoalHandle::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_id != goal_request_id_) {
    return;
  }
  goal_pending_ = false;
  current_goal_handle_.reset();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    last_goal_succeeded_ = true;
    if (!active_path_.poses.empty()) {
      latest_pose_ = active_path_.poses.back();
      has_current_pose_ = true;
    }
    return;
  }

  last_goal_succeeded_ = false;

  if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(logger_, "NavigateThroughPoses goal was canceled");
    return;
  }

  RCLCPP_WARN(
    logger_, "NavigateThroughPoses goal finished with result code %d",
    static_cast<int>(result.code));
}

void SendNavThroughPosesAction::cancelCurrentGoal()
{
  GoalHandle::SharedPtr goal_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++goal_request_id_;
    goal_handle = current_goal_handle_;
    current_goal_handle_.reset();
    goal_pending_ = false;
    last_goal_succeeded_ = false;
  }
  if (goal_handle) {
    action_client_->async_cancel_goal(goal_handle);
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SendNavThroughPosesAction>(
    "SendNavThroughPoses");
}
