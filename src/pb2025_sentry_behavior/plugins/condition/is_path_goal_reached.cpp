#include "pb2025_sentry_behavior/plugins/condition/is_path_goal_reached.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

IsPathGoalReachedCondition::IsPathGoalReachedCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsPathGoalReachedCondition::tickCondition, this), config),
  node_(decision::getNodeFromBlackboard(*this)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.decision_config.path_tolerance", path_tolerance_);
}

BT::NodeStatus IsPathGoalReachedCondition::tickCondition()
{
  auto path = getInput<nav_msgs::msg::Path>("path");
  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if (!path || !current_pose || path->poses.empty()) {
    RCLCPP_DEBUG(logger_, "Path or current pose is not available");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped pose_in_path_frame = *current_pose;
  if (pose_in_path_frame.header.frame_id.empty()) {
    pose_in_path_frame.header.frame_id = path->header.frame_id;
  } else if (!path->header.frame_id.empty() &&
    pose_in_path_frame.header.frame_id != path->header.frame_id)
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        path->header.frame_id, pose_in_path_frame.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped transformed_pose;
      tf2::doTransform(pose_in_path_frame, transformed_pose, transform);
      pose_in_path_frame = transformed_pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "Failed to transform current pose from frame '%s' to path frame '%s': %s",
        pose_in_path_frame.header.frame_id.c_str(), path->header.frame_id.c_str(), ex.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  return decision::isPathGoalReached(pose_in_path_frame, *path, path_tolerance_) ?
           BT::NodeStatus::SUCCESS :
           BT::NodeStatus::FAILURE;
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

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsPathGoalReachedCondition>(
    "IsPathGoalReached");
}
