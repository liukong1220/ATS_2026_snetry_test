#include "pb2025_sentry_behavior/plugins/action/send_nav_through_poses.hpp"

#include <array>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

std::optional<geometry_msgs::msg::PoseStamped>
SendNavThroughPosesAction::readObservedPoseFromBlackboard() const
{
  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if (current_pose) {
    return *current_pose;
  }

  auto root_blackboard = config().blackboard ? config().blackboard->rootBlackboard() : nullptr;
  if (root_blackboard == nullptr) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped pose;
  if (!root_blackboard->get("decision_current_pose", pose)) {
    return std::nullopt;
  }

  return pose;
}

std::optional<geometry_msgs::msg::PoseStamped>
SendNavThroughPosesAction::lookupObservedPoseFromTf(const std::string & target_frame) const
{
  if (target_frame.empty()) {
    return std::nullopt;
  }

  static const std::array<const char *, 2> kBaseFrameCandidates{{"base_footprint", "base_link"}};
  for (const auto * base_frame : kBaseFrameCandidates) {
    try {
      const auto transform = tf_buffer_->lookupTransform(target_frame, base_frame, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return pose;
    } catch (const tf2::TransformException &) {
    }
  }

  return std::nullopt;
}

std::optional<geometry_msgs::msg::PoseStamped>
SendNavThroughPosesAction::transformPoseToPathFrame(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & path_frame) const
{
  if (path_frame.empty()) {
    return pose;
  }

  geometry_msgs::msg::PoseStamped pose_in = pose;
  if (pose_in.header.frame_id.empty() || pose_in.header.frame_id == path_frame) {
    pose_in.header.frame_id = path_frame;
    return pose_in;
  }

  try {
    const auto transform =
      tf_buffer_->lookupTransform(path_frame, pose_in.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped transformed_pose;
    tf2::doTransform(pose_in, transformed_pose, transform);
    return transformed_pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Failed to transform observed pose from frame '%s' to path frame '%s': %s",
      pose_in.header.frame_id.c_str(), path_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

bool SendNavThroughPosesAction::isActiveGoalStillReached(
  const nav_msgs::msg::Path & path, double tolerance) const
{
  if (path.poses.empty()) {
    return false;
  }

  auto observed_pose = readObservedPoseFromBlackboard();
  bool pose_stale = !observed_pose;
  if (observed_pose && pose_timeout_s_ > 0.0) {
    const rclcpp::Time pose_stamp(observed_pose->header.stamp);
    if (pose_stamp.nanoseconds() > 0) {
      pose_stale = (node_->now() - pose_stamp).seconds() > pose_timeout_s_;
    }
  }

  if (pose_stale) {
    observed_pose = lookupObservedPoseFromTf(path.header.frame_id);
  }
  if (!observed_pose) {
    return false;
  }

  const auto pose_in_path_frame =
    transformPoseToPathFrame(*observed_pose, path.header.frame_id);
  if (!pose_in_path_frame) {
    return false;
  }

  return decision::isPathGoalReached(*pose_in_path_frame, path, tolerance);
}

SendNavThroughPosesAction::SendNavThroughPosesAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  node_(decision::getNodeFromBlackboard(*this)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false))
{
  logger_ = node_->get_logger();
  action_name_ = "/navigate_through_poses";
  node_->get_parameter("decision.decision_config.nav2_action_server", action_name_);
  node_->get_parameter("decision.decision_config.nav2_to_pose_action_server", action_to_pose_name_);
  node_->get_parameter(
    "decision.decision_config.goal_position_tolerance", path_compare_tolerance_);
  node_->get_parameter(
    "decision.decision_config.path_goal_reached_tolerance", path_goal_reached_tolerance_);
  node_->get_parameter(
    "decision.decision_config.action_server_wait_timeout_s", action_server_wait_timeout_s_);
  node_->get_parameter(
    "decision.decision_config.active_goal_hold_tolerance", active_goal_hold_tolerance_);
  node_->get_parameter(
    "decision.decision_config.active_goal_min_resend_interval_s",
    active_goal_min_resend_interval_s_);
  node_->get_parameter(
    "decision.decision_config.vision_active_goal_hold_tolerance",
    vision_active_goal_hold_tolerance_);
  node_->get_parameter(
    "decision.decision_config.vision_active_goal_min_resend_interval_s",
    vision_active_goal_min_resend_interval_s_);
  node_->get_parameter("decision.pose.timeout_s", pose_timeout_s_);

  action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, action_name_);
  action_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_to_pose_name_);
}

BT::PortsList SendNavThroughPosesAction::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Decision path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{@decision_current_pose}", "Current navigation feedback pose"),
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

  const bool is_single_pose_path = path->poses.size() == 1;
  const auto action_wait_timeout =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(action_server_wait_timeout_s_));
  const bool action_server_ready = is_single_pose_path ?
    action_to_pose_client_->wait_for_action_server(action_wait_timeout) :
    action_client_->wait_for_action_server(action_wait_timeout);
  if (!action_server_ready)
  {
    RCLCPP_ERROR(
      logger_, "Action server %s is not available",
      (is_single_pose_path ? action_to_pose_name_ : action_name_).c_str());
    return BT::NodeStatus::FAILURE;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto observed_pose = readObservedPoseFromBlackboard();
    if (observed_pose) {
      latest_pose_ = *observed_pose;
      has_current_pose_ = true;
    }

    const bool same_path =
      decision::pathEquivalent(active_path_, *path, path_compare_tolerance_);
    // 视觉跟随目标本身就要求更高频地围绕敌方实时刷新，因此不能完全沿用普通巡逻/
    // 退防路径的“近似目标保持”阈值。否则圆周上的新最近点会被误当成旧路径抖动吞掉。
    const bool is_vision_follow_path = active_path_.poses.size() == 1 && path->poses.size() == 1;
    const double hold_tolerance = is_vision_follow_path ?
      vision_active_goal_hold_tolerance_ : active_goal_hold_tolerance_;
    const double min_resend_interval_s = is_vision_follow_path ?
      vision_active_goal_min_resend_interval_s_ : active_goal_min_resend_interval_s_;
    const bool near_active_path =
      decision::pathEquivalent(active_path_, *path, hold_tolerance);
    const bool active_goal_still_reached =
      same_path && last_goal_succeeded_ && !goal_pending_ && !current_goal_handle_ &&
      ((has_current_pose_ &&
      decision::isPathGoalReached(latest_pose_, *path, path_goal_reached_tolerance_)) ||
      isActiveGoalStillReached(*path, path_goal_reached_tolerance_));
    const bool same_path_goal_recently_succeeded =
      same_path && last_goal_succeeded_ && !goal_pending_ && !current_goal_handle_ &&
      !has_current_pose_;
    setOutput(
      "goal_succeeded",
      active_goal_still_reached || same_path_goal_recently_succeeded);

    if (active_goal_still_reached || same_path_goal_recently_succeeded) {
      return BT::NodeStatus::SUCCESS;
    }

    if ((goal_pending_ || current_goal_handle_) && same_path)
    {
      return BT::NodeStatus::SUCCESS;
    }

    if ((goal_pending_ || current_goal_handle_) && near_active_path && has_last_goal_sent_at_) {
      const double since_last_send_s = (node_->now() - last_goal_sent_at_).seconds();
      if (since_last_send_s < min_resend_interval_s) {
        RCLCPP_DEBUG_THROTTLE(
          logger_, *node_->get_clock(), 1000,
          "Keep active NavigateThroughPoses goal instead of preempting a near-identical path "
          "(dt=%.2fs hold_tol=%.2fm resend_interval=%.2fs vision=%d)",
          since_last_send_s, hold_tolerance, min_resend_interval_s,
          static_cast<int>(is_vision_follow_path));
        return BT::NodeStatus::SUCCESS;
      }
    }
  }

  cancelCurrentGoal();

  const auto now = node_->now();

  std::uint64_t request_id = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    request_id = ++goal_request_id_;
    goal_pending_ = true;
    last_goal_succeeded_ = false;
    active_path_ = *path;
    last_goal_sent_at_ = now;
    has_last_goal_sent_at_ = true;
  }
  setOutput("goal_succeeded", false);

  if (is_single_pose_path) {
    NavigateToPose::Goal goal;
    goal.pose = path->poses.front();
    goal.pose.header.stamp = now;
    if (goal.pose.header.frame_id.empty()) {
      goal.pose.header.frame_id = "map";
    }

    const auto & pose = goal.pose.pose.position;
    RCLCPP_INFO(
      logger_,
      "Send NavigateToPose goal for single-pose vision path: pose=(%.2f, %.2f)",
      pose.x, pose.y);

    auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this, request_id](const GoalHandleToPose::SharedPtr handle) {
        goalResponseToPoseCallback(request_id, handle);
      };
    send_goal_options.feedback_callback =
      [this, request_id](
        GoalHandleToPose::SharedPtr handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
          feedbackToPoseCallback(request_id, handle, feedback);
        };
    send_goal_options.result_callback =
      [this, request_id](const GoalHandleToPose::WrappedResult & result) {
        resultToPoseCallback(request_id, result);
      };

    action_to_pose_client_->async_send_goal(goal, send_goal_options);
    return BT::NodeStatus::SUCCESS;
  }

  NavigateThroughPoses::Goal goal;
  goal.poses = path->poses;
  for (auto & pose : goal.poses) {
    pose.header.stamp = now;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = "map";
    }
  }

  const auto & first_pose = goal.poses.front().pose.position;
  const auto & last_pose = goal.poses.back().pose.position;
  RCLCPP_INFO(
    logger_,
    "Send NavigateThroughPoses goal with %zu poses: first=(%.2f, %.2f) last=(%.2f, %.2f)",
    goal.poses.size(), first_pose.x, first_pose.y, last_pose.x, last_pose.y);

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
  current_goal_to_pose_handle_.reset();
  if (!goal_handle) {
    last_goal_succeeded_ = false;
    RCLCPP_ERROR(logger_, "NavigateThroughPoses goal was rejected by server");
  }
}

void SendNavThroughPosesAction::goalResponseToPoseCallback(
  std::uint64_t request_id, const GoalHandleToPose::SharedPtr & goal_handle)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_id != goal_request_id_) {
    return;
  }
  goal_pending_ = false;
  current_goal_to_pose_handle_ = goal_handle;
  current_goal_handle_.reset();
  if (!goal_handle) {
    last_goal_succeeded_ = false;
    RCLCPP_ERROR(logger_, "NavigateToPose goal was rejected by server");
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

void SendNavThroughPosesAction::feedbackToPoseCallback(
  std::uint64_t request_id, GoalHandleToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
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

void SendNavThroughPosesAction::resultToPoseCallback(
  std::uint64_t request_id, const GoalHandleToPose::WrappedResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (request_id != goal_request_id_) {
    return;
  }
  goal_pending_ = false;
  current_goal_to_pose_handle_.reset();
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
    RCLCPP_INFO(logger_, "NavigateToPose goal was canceled");
    return;
  }

  RCLCPP_WARN(
    logger_, "NavigateToPose goal finished with result code %d",
    static_cast<int>(result.code));
}

void SendNavThroughPosesAction::cancelCurrentGoal()
{
  GoalHandle::SharedPtr goal_handle;
  GoalHandleToPose::SharedPtr goal_to_pose_handle;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++goal_request_id_;
    goal_handle = current_goal_handle_;
    goal_to_pose_handle = current_goal_to_pose_handle_;
    current_goal_handle_.reset();
    current_goal_to_pose_handle_.reset();
    goal_pending_ = false;
    last_goal_succeeded_ = false;
  }
  if (goal_handle) {
    action_client_->async_cancel_goal(goal_handle);
  }
  if (goal_to_pose_handle) {
    action_to_pose_client_->async_cancel_goal(goal_to_pose_handle);
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SendNavThroughPosesAction>(
    "SendNavThroughPoses");
}
