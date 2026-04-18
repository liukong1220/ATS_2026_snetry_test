#include "pb2025_sentry_behavior/plugins/action/select_vision_follow_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "sp_msgs/msg/vision_target_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

constexpr double kPositionEpsilon = 1e-6;

bool isFinitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double planarDistance(
  const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Point sampleCirclePoint(
  const geometry_msgs::msg::Point & center, double radius, double angle)
{
  geometry_msgs::msg::Point point;
  point.x = center.x + radius * std::cos(angle);
  point.y = center.y + radius * std::sin(angle);
  point.z = center.z;
  return point;
}

std::vector<double> buildAngleOffsets(int sample_count, double max_offset)
{
  const int effective_samples = std::max(2, sample_count / 2);

  std::vector<double> offsets;
  offsets.reserve(static_cast<std::size_t>(effective_samples * 2 + 1));
  offsets.push_back(0.0);

  const double step = max_offset / static_cast<double>(effective_samples);
  for (int i = 1; i <= effective_samples; ++i) {
    const double offset = step * static_cast<double>(i);
    offsets.push_back(offset);
    offsets.push_back(-offset);
  }
  return offsets;
}

std::vector<geometry_msgs::msg::Point> buildArcPoints(
  const geometry_msgs::msg::Point & center, double radius, double base_angle, double arc_half_angle,
  int segments)
{
  const int clamped_segments = std::max(12, segments);
  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(static_cast<std::size_t>(clamped_segments + 1));

  for (int i = 0; i <= clamped_segments; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(clamped_segments);
    const double angle =
      base_angle - arc_half_angle + (2.0 * arc_half_angle * ratio);
    points.push_back(sampleCirclePoint(center, radius, angle));
  }
  return points;
}

bool isTraversable(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Point & point,
  int occupied_threshold)
{
  if (costmap.data.empty() || costmap.info.width == 0 || costmap.info.height == 0 ||
    costmap.info.resolution <= 0.0)
  {
    return true;
  }

  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;
  const double resolution = static_cast<double>(costmap.info.resolution);

  const auto mx = static_cast<int>(std::floor((point.x - origin_x) / resolution));
  const auto my = static_cast<int>(std::floor((point.y - origin_y) / resolution));

  if (mx < 0 || my < 0 || mx >= static_cast<int>(costmap.info.width) ||
    my >= static_cast<int>(costmap.info.height))
  {
    return false;
  }

  const auto index =
    static_cast<std::size_t>(my) * static_cast<std::size_t>(costmap.info.width) +
    static_cast<std::size_t>(mx);
  const auto cell = costmap.data[index];
  return cell >= 0 && cell < occupied_threshold;
}

geometry_msgs::msg::PoseStamped buildFacingPose(
  const geometry_msgs::msg::Point & goal_point, const geometry_msgs::msg::Point & target_point,
  const std::string & frame_id, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose.position = goal_point;

  tf2::Quaternion quaternion;
  quaternion.setRPY(
    0.0, 0.0, std::atan2(target_point.y - goal_point.y, target_point.x - goal_point.x));
  pose.pose.orientation = tf2::toMsg(quaternion);

  return pose;
}

}  // namespace

namespace pb2025_sentry_behavior
{

SelectVisionFollowPathAction::SelectVisionFollowPathAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  node_(decision::getNodeFromBlackboard(*this)),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node_->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false))
{
  logger_ = node_->get_logger();
  std::string visualization_topic = "decision/vision_follow_markers";
  node_->get_parameter("decision.vision.visualization_topic", visualization_topic);
  visualization_publisher_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);
}

BT::NodeStatus SelectVisionFollowPathAction::tick()
{
  auto vision_target = getInput<sp_msgs::msg::VisionTargetMsg>("key_port");
  if (!vision_target) {
    resetCachedPath();
    clearVisualization();
    RCLCPP_DEBUG(logger_, "SelectVisionFollowPath did not receive a vision target");
    return BT::NodeStatus::FAILURE;
  }

  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if (!current_pose) {
    resetCachedPath();
    clearVisualization();
    RCLCPP_WARN(logger_, "Current pose is unavailable, cannot build a vision follow path");
    return BT::NodeStatus::FAILURE;
  }

  double pose_timeout_s = 0.5;
  node_->get_parameter("decision.pose.timeout_s", pose_timeout_s);
  const rclcpp::Time current_pose_stamp(current_pose->header.stamp);
  if (pose_timeout_s > 0.0 && current_pose_stamp.nanoseconds() > 0) {
    const auto age_s = (node_->now() - current_pose_stamp).seconds();
    if (age_s > pose_timeout_s) {
      resetCachedPath();
      clearVisualization();
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "Current pose is stale, skip vision follow path: age=%.3fs timeout=%.3fs",
        age_s, pose_timeout_s);
      return BT::NodeStatus::FAILURE;
    }
  }

  auto costmap = getInput<nav_msgs::msg::OccupancyGrid>("current_costmap");
  const std::string costmap_frame =
    (costmap && !costmap->header.frame_id.empty()) ? costmap->header.frame_id : "";
  const auto planning_frame = resolvePlanningFrame(costmap_frame, *current_pose, *vision_target);

  const auto target_point = transformTargetPointToFrame(*vision_target, planning_frame);
  if (!target_point) {
    resetCachedPath();
    clearVisualization();
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Vision target_position_map is invalid or not transformable into planning frame '%s'",
      planning_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto transformed_current_pose = transformPoseToFrame(*current_pose, planning_frame);
  if (!transformed_current_pose) {
    resetCachedPath();
    clearVisualization();
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Failed to transform current pose into planning frame '%s'", planning_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  double attack_radius = 2.0;
  int occupied_threshold = 50;
  int sample_count = 16;
  double follow_arc_half_angle_deg = 90.0;
  double min_replan_interval_s = 0.4;
  double min_goal_shift_m = 0.35;
  node_->get_parameter("decision.vision.attack_radius", attack_radius);
  node_->get_parameter("decision.vision.follow_occupied_threshold", occupied_threshold);
  node_->get_parameter("decision.vision.follow_sample_count", sample_count);
  node_->get_parameter("decision.vision.follow_arc_half_angle_deg", follow_arc_half_angle_deg);
  node_->get_parameter("decision.vision.min_replan_interval_s", min_replan_interval_s);
  node_->get_parameter("decision.vision.min_goal_shift_m", min_goal_shift_m);

  getInput("attack_radius", attack_radius);
  getInput("occupied_threshold", occupied_threshold);
  getInput("sample_count", sample_count);

  attack_radius = std::max(0.1, attack_radius);
  occupied_threshold = std::clamp(occupied_threshold, 1, 100);
  sample_count = std::max(4, sample_count);
  const double arc_half_angle = std::clamp(
    follow_arc_half_angle_deg * M_PI / 180.0, M_PI / 18.0, M_PI);

  const auto & current_position = transformed_current_pose->pose.position;
  double preferred_angle = 0.0;
  if (planarDistance(current_position, *target_point) > kPositionEpsilon) {
    preferred_angle = std::atan2(
      current_position.y - target_point->y, current_position.x - target_point->x);
  }

  geometry_msgs::msg::Point selected_point =
    sampleCirclePoint(*target_point, attack_radius, preferred_angle);

  const bool use_costmap_screening =
    costmap && !costmap->data.empty() && !costmap->header.frame_id.empty() &&
    costmap->header.frame_id == planning_frame;
  if (costmap && !costmap->data.empty() && !costmap->header.frame_id.empty() &&
    costmap->header.frame_id != planning_frame)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Skip costmap screening because costmap frame '%s' does not match planning frame '%s'",
      costmap->header.frame_id.c_str(), planning_frame.c_str());
  }

  if (use_costmap_screening) {
    static constexpr std::array<double, 5> kRadiusScales{{1.0, 0.85, 0.7, 0.55, 0.4}};
    const auto angle_offsets = buildAngleOffsets(sample_count, arc_half_angle);

    bool found_candidate = false;
    for (const auto radius_scale : kRadiusScales) {
      const double radius = attack_radius * radius_scale;
      for (const auto angle_offset : angle_offsets) {
        const auto candidate =
          sampleCirclePoint(*target_point, radius, preferred_angle + angle_offset);
        if (!isTraversable(*costmap, candidate, occupied_threshold)) {
          continue;
        }
        selected_point = candidate;
        found_candidate = true;
        break;
      }
      if (found_candidate) {
        break;
      }
    }

    if (!found_candidate) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "No free candidate found on the vision follow ring, fallback to preferred point");
    }
  }

  const auto now = node_->now();
  publishVisualization(
    planning_frame, current_position, *target_point, selected_point, attack_radius, preferred_angle,
    arc_half_angle, sample_count);

  if (shouldReuseCachedPath(
      now, planning_frame, selected_point, min_replan_interval_s, min_goal_shift_m))
  {
    setOutput("path", last_path_);
    RCLCPP_DEBUG_THROTTLE(
      logger_, *node_->get_clock(), 1000,
      "Reuse cached vision follow path in frame '%s' (target=(%.2f, %.2f) selected_goal=(%.2f, %.2f))",
      planning_frame.c_str(), target_point->x, target_point->y, last_selected_goal_.x,
      last_selected_goal_.y);
    return BT::NodeStatus::SUCCESS;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = planning_frame;
  path.poses.push_back(
    buildFacingPose(selected_point, *target_point, planning_frame, now));
  cachePath(*target_point, selected_point, path, planning_frame, now);

  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 2000,
    "Vision follow target=(%.2f, %.2f) selected_goal=(%.2f, %.2f) attack_radius=%.2f arc_half_angle_deg=%.1f frame=%s",
    target_point->x, target_point->y, selected_point.x, selected_point.y, attack_radius,
    follow_arc_half_angle_deg, planning_frame.c_str());

  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

std::string SelectVisionFollowPathAction::resolvePlanningFrame(
  const std::string & costmap_frame, const geometry_msgs::msg::PoseStamped & current_pose,
  const sp_msgs::msg::VisionTargetMsg & vision_target) const
{
  if (!costmap_frame.empty()) {
    return costmap_frame;
  }

  std::string expected_frame = "map";
  node_->get_parameter("decision.pose.expected_frame", expected_frame);
  if (!expected_frame.empty()) {
    return expected_frame;
  }

  if (!current_pose.header.frame_id.empty()) {
    return current_pose.header.frame_id;
  }

  if (vision_target.has_target_position_map && !vision_target.target_position_map_frame.empty()) {
    return vision_target.target_position_map_frame;
  }

  return "map";
}

std::optional<geometry_msgs::msg::Point> SelectVisionFollowPathAction::transformTargetPointToFrame(
  const sp_msgs::msg::VisionTargetMsg & vision_target, const std::string & target_frame) const
{
  if (!vision_target.has_target_position_map || !isFinitePoint(vision_target.target_position_map) ||
    vision_target.target_position_map_frame.empty())
  {
    return std::nullopt;
  }

  if (vision_target.target_position_map_frame == target_frame) {
    return vision_target.target_position_map;
  }

  geometry_msgs::msg::PointStamped point_in;
  point_in.header.stamp = rclcpp::Time(vision_target.timestamp);
  if (rclcpp::Time(point_in.header.stamp).nanoseconds() <= 0) {
    point_in.header.stamp = node_->now();
  }
  point_in.header.frame_id = vision_target.target_position_map_frame;
  point_in.point = vision_target.target_position_map;

  try {
    const auto transform =
      tf_buffer_->lookupTransform(target_frame, point_in.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PointStamped point_out;
    tf2::doTransform(point_in, point_out, transform);
    return isFinitePoint(point_out.point) ?
             std::optional<geometry_msgs::msg::Point>(point_out.point) :
             std::nullopt;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Failed to transform vision target from frame '%s' to '%s': %s",
      point_in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::PoseStamped> SelectVisionFollowPathAction::transformPoseToFrame(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame) const
{
  if (!isFinitePoint(pose.pose.position)) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped pose_in = pose;
  if (pose_in.header.frame_id.empty()) {
    pose_in.header.frame_id = target_frame;
    return pose_in;
  }

  if (pose_in.header.frame_id == target_frame) {
    return pose_in;
  }

  try {
    const auto transform =
      tf_buffer_->lookupTransform(target_frame, pose_in.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped pose_out;
    tf2::doTransform(pose_in, pose_out, transform);
    return isFinitePoint(pose_out.pose.position) ?
             std::optional<geometry_msgs::msg::PoseStamped>(pose_out) :
             std::nullopt;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Failed to transform current pose from frame '%s' to '%s': %s",
      pose_in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
    return std::nullopt;
  }
}

bool SelectVisionFollowPathAction::shouldReuseCachedPath(
  const rclcpp::Time & now, const std::string & planning_frame,
  const geometry_msgs::msg::Point & selected_goal, double min_replan_interval_s,
  double min_goal_shift_m) const
{
  if (!has_cached_path_ || !last_plan_time_ || last_path_.poses.empty()) {
    return false;
  }

  if (last_plan_frame_id_ != planning_frame) {
    return false;
  }

  if ((now - *last_plan_time_).seconds() < min_replan_interval_s) {
    return true;
  }

  return planarDistance(selected_goal, last_selected_goal_) < min_goal_shift_m;
}

void SelectVisionFollowPathAction::cachePath(
  const geometry_msgs::msg::Point & target_point,
  const geometry_msgs::msg::Point & selected_goal, const nav_msgs::msg::Path & path,
  const std::string & planning_frame, const rclcpp::Time & now)
{
  last_target_point_ = target_point;
  last_selected_goal_ = selected_goal;
  last_path_ = path;
  last_plan_time_ = now;
  last_plan_frame_id_ = planning_frame;
  has_cached_path_ = true;
}

void SelectVisionFollowPathAction::publishVisualization(
  const std::string & planning_frame, const geometry_msgs::msg::Point & current_position,
  const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & selected_goal,
  double attack_radius, double preferred_angle, double arc_half_angle, int sample_count)
{
  bool visualization_enabled = true;
  node_->get_parameter("decision.vision.visualization_enabled", visualization_enabled);
  if (!visualization_enabled || !visualization_publisher_) {
    return;
  }

  const auto stamp = node_->now();
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker target_marker;
  target_marker.header.frame_id = planning_frame;
  target_marker.header.stamp = stamp;
  target_marker.ns = "vision_follow";
  target_marker.id = 0;
  target_marker.type = visualization_msgs::msg::Marker::SPHERE;
  target_marker.action = visualization_msgs::msg::Marker::ADD;
  target_marker.pose.orientation.w = 1.0;
  target_marker.pose.position = target_point;
  target_marker.scale.x = 0.22;
  target_marker.scale.y = 0.22;
  target_marker.scale.z = 0.22;
  target_marker.color.r = 1.0F;
  target_marker.color.g = 0.15F;
  target_marker.color.b = 0.15F;
  target_marker.color.a = 0.95F;
  markers.markers.push_back(target_marker);

  visualization_msgs::msg::Marker arc_marker;
  arc_marker.header.frame_id = planning_frame;
  arc_marker.header.stamp = stamp;
  arc_marker.ns = "vision_follow";
  arc_marker.id = 1;
  arc_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  arc_marker.action = visualization_msgs::msg::Marker::ADD;
  arc_marker.pose.orientation.w = 1.0;
  arc_marker.scale.x = 0.06;
  arc_marker.color.r = 0.15F;
  arc_marker.color.g = 0.85F;
  arc_marker.color.b = 1.0F;
  arc_marker.color.a = 0.95F;
  arc_marker.points = buildArcPoints(
    target_point, attack_radius, preferred_angle, arc_half_angle, std::max(24, sample_count));
  markers.markers.push_back(arc_marker);

  visualization_msgs::msg::Marker selected_goal_marker;
  selected_goal_marker.header.frame_id = planning_frame;
  selected_goal_marker.header.stamp = stamp;
  selected_goal_marker.ns = "vision_follow";
  selected_goal_marker.id = 2;
  selected_goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  selected_goal_marker.action = visualization_msgs::msg::Marker::ADD;
  selected_goal_marker.pose.orientation.w = 1.0;
  selected_goal_marker.pose.position = selected_goal;
  selected_goal_marker.scale.x = 0.24;
  selected_goal_marker.scale.y = 0.24;
  selected_goal_marker.scale.z = 0.24;
  selected_goal_marker.color.r = 0.2F;
  selected_goal_marker.color.g = 1.0F;
  selected_goal_marker.color.b = 0.25F;
  selected_goal_marker.color.a = 0.98F;
  markers.markers.push_back(selected_goal_marker);

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = planning_frame;
  path_marker.header.stamp = stamp;
  path_marker.ns = "vision_follow";
  path_marker.id = 3;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.05;
  path_marker.color.r = 1.0F;
  path_marker.color.g = 0.85F;
  path_marker.color.b = 0.1F;
  path_marker.color.a = 0.95F;
  path_marker.points.push_back(current_position);
  path_marker.points.push_back(selected_goal);
  markers.markers.push_back(path_marker);

  visualization_publisher_->publish(markers);
}

void SelectVisionFollowPathAction::clearVisualization()
{
  bool visualization_enabled = true;
  node_->get_parameter("decision.vision.visualization_enabled", visualization_enabled);
  if (!visualization_enabled || !visualization_publisher_) {
    return;
  }

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(clear_marker);
  visualization_publisher_->publish(markers);
}

void SelectVisionFollowPathAction::resetCachedPath()
{
  has_cached_path_ = false;
  last_plan_time_.reset();
  last_plan_frame_id_.clear();
  last_path_ = nav_msgs::msg::Path{};
  last_target_point_ = geometry_msgs::msg::Point{};
  last_selected_goal_ = geometry_msgs::msg::Point{};
}

BT::PortsList SelectVisionFollowPathAction::providedPorts()
{
  return {
    BT::InputPort<sp_msgs::msg::VisionTargetMsg>(
      "key_port", "{@sp_vision_target}", "Vision fusion message on blackboard"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "current_pose", "{@decision_current_pose}", "Current navigation pose"),
    BT::InputPort<nav_msgs::msg::OccupancyGrid>(
      "current_costmap", "{@nav_globalCostmap}", "Global costmap used for point screening"),
    BT::InputPort<double>(
      "attack_radius", 2.0, "Desired follow radius around the vision target"),
    BT::InputPort<int>(
      "occupied_threshold", 50, "Costmap cells at or above this value are treated as blocked"),
    BT::InputPort<int>(
      "sample_count", 16, "Number of angular samples for the follow ring"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Vision follow path")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::SelectVisionFollowPathAction>(
    "SelectVisionFollowPath");
}
