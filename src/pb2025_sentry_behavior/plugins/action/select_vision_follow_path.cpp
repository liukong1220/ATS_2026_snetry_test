#include "pb2025_sentry_behavior/plugins/action/select_vision_follow_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "sp_msgs/msg/vision_target_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

constexpr double kPositionEpsilon = 1e-6;

bool isFinitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool hasUsableMapPosition(const geometry_msgs::msg::Point & point)
{
  if (!isFinitePoint(point)) {
    return false;
  }

  // 视觉侧尚未接地图坐标时，当前约定会保持零值。
  return std::fabs(point.x) + std::fabs(point.y) + std::fabs(point.z) > kPositionEpsilon;
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

std::vector<double> buildAngleOffsets(int sample_count)
{
  const int effective_samples = std::max(2, sample_count / 2);

  std::vector<double> offsets;
  offsets.reserve(static_cast<std::size_t>(effective_samples * 2 + 1));
  offsets.push_back(0.0);

  const double step = M_PI / static_cast<double>(effective_samples);
  for (int i = 1; i <= effective_samples; ++i) {
    const double offset = step * static_cast<double>(i);
    offsets.push_back(offset);
    offsets.push_back(-offset);
  }
  return offsets;
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
: BT::SyncActionNode(name, config), node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
}

BT::NodeStatus SelectVisionFollowPathAction::tick()
{
  auto vision_target = getInput<sp_msgs::msg::VisionTargetMsg>("key_port");
  if (!vision_target) {
    RCLCPP_DEBUG(logger_, "SelectVisionFollowPath did not receive a vision target");
    return BT::NodeStatus::FAILURE;
  }

  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");
  if (!current_pose) {
    RCLCPP_WARN(logger_, "Current pose is unavailable, cannot build a vision follow path");
    return BT::NodeStatus::FAILURE;
  }

  const auto & target_point = vision_target->target_position_map;
  if (!hasUsableMapPosition(target_point)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Vision target_position_map is not ready, skip vision follow path");
    return BT::NodeStatus::FAILURE;
  }

  double attack_radius = 2.0;
  int occupied_threshold = 50;
  int sample_count = 16;
  node_->get_parameter("decision.vision.attack_radius", attack_radius);
  node_->get_parameter("decision.vision.follow_occupied_threshold", occupied_threshold);
  node_->get_parameter("decision.vision.follow_sample_count", sample_count);

  getInput("attack_radius", attack_radius);
  getInput("occupied_threshold", occupied_threshold);
  getInput("sample_count", sample_count);

  attack_radius = std::max(0.1, attack_radius);
  occupied_threshold = std::clamp(occupied_threshold, 1, 100);
  sample_count = std::max(4, sample_count);

  const auto & current_position = current_pose->pose.position;
  double preferred_angle = 0.0;
  if (planarDistance(current_position, target_point) > kPositionEpsilon) {
    preferred_angle = std::atan2(
      current_position.y - target_point.y, current_position.x - target_point.x);
  }

  geometry_msgs::msg::Point selected_point =
    sampleCirclePoint(target_point, attack_radius, preferred_angle);

  auto costmap = getInput<nav_msgs::msg::OccupancyGrid>("current_costmap");
  if (costmap && !costmap->data.empty()) {
    static constexpr std::array<double, 5> kRadiusScales{{1.0, 0.85, 0.7, 0.55, 0.4}};
    const auto angle_offsets = buildAngleOffsets(sample_count);

    bool found_candidate = false;
    for (const auto radius_scale : kRadiusScales) {
      const double radius = attack_radius * radius_scale;
      for (const auto angle_offset : angle_offsets) {
        const auto candidate =
          sampleCirclePoint(target_point, radius, preferred_angle + angle_offset);
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

  std::string frame_id = "map";
  if (costmap && !costmap->header.frame_id.empty()) {
    frame_id = costmap->header.frame_id;
  } else if (!current_pose->header.frame_id.empty()) {
    frame_id = current_pose->header.frame_id;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = frame_id;
  path.poses.push_back(
    buildFacingPose(selected_point, target_point, frame_id, node_->now()));

  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 2000,
    "Vision follow target=(%.2f, %.2f) selected_goal=(%.2f, %.2f) attack_radius=%.2f",
    target_point.x, target_point.y, selected_point.x, selected_point.y, attack_radius);

  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
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
