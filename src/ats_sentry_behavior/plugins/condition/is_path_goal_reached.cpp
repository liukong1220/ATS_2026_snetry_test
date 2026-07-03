#include "ats_sentry_behavior/plugins/condition/is_path_goal_reached.hpp"

#include <array>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ats_sentry_behavior
{

std::optional<geometry_msgs::msg::PoseStamped>
IsPathGoalReachedCondition::readObservedPoseFromBlackboard() const
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
IsPathGoalReachedCondition::lookupObservedPoseFromTf(const std::string & target_frame) const
{
  if (target_frame.empty()) {
    return std::nullopt;
  }

  static constexpr std::array<const char *, 2> kBaseFrameCandidates{{"base_footprint", "base_link"}};
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
IsPathGoalReachedCondition::transformPoseToPathFrame(
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
      "Failed to transform current pose from frame '%s' to path frame '%s': %s",
      pose_in.header.frame_id.c_str(), path_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::Point>
IsPathGoalReachedCondition::extractGoalPoint(const nav_msgs::msg::Path & path) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }
  return path.poses.back().pose.position;
}

IsPathGoalReachedCondition::IsPathGoalReachedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPathGoalReachedCondition::tickCondition, this), config),
  node_(decision::getNodeFromBlackboard(*this)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.decision_config.path_goal_reached_tolerance", path_tolerance_);
  node_->get_parameter("decision.pose.timeout_s", pose_timeout_s_);
}

BT::NodeStatus IsPathGoalReachedCondition::tickCondition()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  const auto goal_point = path ? extractGoalPoint(*path) : std::nullopt;

  auto goal_succeeded = getInput<bool>("goal_succeeded");
  if (goal_succeeded && !*goal_succeeded) {
    last_succeeded_goal_point_.reset();
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
    observed_pose = lookupObservedPoseFromTf(path ? path->header.frame_id : std::string{});
  }

  if (!path || !observed_pose || path->poses.empty()) {
    RCLCPP_DEBUG(logger_, "Path or current pose is not available");
    return BT::NodeStatus::FAILURE;
  }

  const auto pose_in_path_frame = transformPoseToPathFrame(*observed_pose, path->header.frame_id);
  if (!pose_in_path_frame) {
    return BT::NodeStatus::FAILURE;
  }

  const bool reached = decision::isPathGoalReached(*pose_in_path_frame, *path, path_tolerance_);
  if (reached && goal_point) {
    last_succeeded_goal_point_ = *goal_point;
  } else if (!reached) {
    last_succeeded_goal_point_.reset();
  }

  return reached ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsPathGoalReachedCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Current decision path"),
    BT::InputPort<bool>(
      "goal_succeeded", "Whether NavigateThroughPoses already reported success for this path"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{decision_current_pose}", "Current navigation feedback pose")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::IsPathGoalReachedCondition>(
    "IsPathGoalReached");
}
