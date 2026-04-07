#ifndef PB2025_SENTRY_BEHAVIOR__DECISION_UTILS_HPP_
#define PB2025_SENTRY_BEHAVIOR__DECISION_UTILS_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/tree_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior::decision
{

enum class GameTimeStage
{
  kUnknown = 0,
  kAbundant,
  kNormal,
  kTense,
  kCritical
};

enum class HpBand
{
  kNormal = 0,
  kLow,
  kCritical
};

struct TimeThresholds
{
  int abundant = 300;
  int normal = 240;
  int tense = 120;
  int critical = 40;
};

struct HpStageThresholds
{
  int high = 400;
  int medium = 250;
  int low = 150;
};

struct HpThresholds
{
  HpStageThresholds abundant;
  HpStageThresholds normal;
  HpStageThresholds tense;
  HpStageThresholds critical;
};

inline rclcpp::Node::SharedPtr getNodeFromBlackboard(const BT::TreeNode & tree_node)
{
  auto root_blackboard = tree_node.config().blackboard->rootBlackboard();
  if (root_blackboard == nullptr) {
    throw std::runtime_error("BehaviorTree root blackboard is not available");
  }

  rclcpp::Node::SharedPtr node;
  if (!root_blackboard->get("node", node)) {
    throw std::runtime_error("ROS node handle is not stored in the behavior tree blackboard");
  }
  if (!node) {
    throw std::runtime_error("ROS node handle is not stored in the behavior tree blackboard");
  }
  return node;
}

inline geometry_msgs::msg::PoseStamped makePoseStamped(
  const geometry_msgs::msg::Point & point, const std::string & frame_id = "map")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position = point;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.0);
  pose.pose.orientation = tf2::toMsg(quaternion);
  return pose;
}

inline std::vector<geometry_msgs::msg::Point> buildGoalPoints(
  const std::vector<double> & xs, const std::vector<double> & ys, const std::vector<double> & zs)
{
  if (xs.size() != ys.size() || xs.size() != zs.size()) {
    throw std::runtime_error("Decision goal_points.x/y/z size mismatch");
  }

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(xs.size());
  for (std::size_t i = 0; i < xs.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = xs[i];
    point.y = ys[i];
    point.z = zs[i];
    points.push_back(point);
  }
  return points;
}

inline std::size_t validateIndex(std::size_t index, std::size_t total_points, const char * label)
{
  if (index >= total_points) {
    throw std::runtime_error(std::string(label) + " index is out of range");
  }
  return index;
}

inline nav_msgs::msg::Path buildPathFromIndices(
  const std::vector<geometry_msgs::msg::Point> & goal_points, const std::vector<std::size_t> & indices,
  const std::string & frame_id = "map")
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.poses.reserve(indices.size());

  for (const auto index : indices) {
    validateIndex(index, goal_points.size(), "goal_points");
    path.poses.push_back(makePoseStamped(goal_points[index], frame_id));
  }
  return path;
}

inline GameTimeStage classifyGameTimeStage(
  int remain_time, const TimeThresholds & thresholds)
{
  if (remain_time > thresholds.normal && remain_time <= thresholds.abundant) {
    return GameTimeStage::kAbundant;
  }
  if (remain_time > thresholds.tense && remain_time <= thresholds.normal) {
    return GameTimeStage::kNormal;
  }
  if (remain_time > thresholds.critical && remain_time <= thresholds.tense) {
    return GameTimeStage::kTense;
  }
  if (remain_time > 0 && remain_time <= thresholds.critical) {
    return GameTimeStage::kCritical;
  }
  return GameTimeStage::kUnknown;
}

inline HpStageThresholds selectHpThresholds(
  GameTimeStage stage, const HpThresholds & thresholds)
{
  switch (stage) {
    case GameTimeStage::kAbundant:
      return thresholds.abundant;
    case GameTimeStage::kNormal:
      return thresholds.normal;
    case GameTimeStage::kTense:
      return thresholds.tense;
    case GameTimeStage::kCritical:
      return thresholds.critical;
    case GameTimeStage::kUnknown:
    default:
      return thresholds.normal;
  }
}

inline HpBand classifyHpBand(int current_hp, const HpStageThresholds & thresholds)
{
  if (current_hp > thresholds.medium && current_hp <= thresholds.high) {
    return HpBand::kNormal;
  }
  if (current_hp > thresholds.low && current_hp <= thresholds.medium) {
    return HpBand::kLow;
  }
  return HpBand::kCritical;
}

inline double squaredDistance(
  const geometry_msgs::msg::Point & lhs, const geometry_msgs::msg::Point & rhs)
{
  const double dx = lhs.x - rhs.x;
  const double dy = lhs.y - rhs.y;
  const double dz = lhs.z - rhs.z;
  return dx * dx + dy * dy + dz * dz;
}

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool isPathGoalReached(
  const geometry_msgs::msg::PoseStamped & current_pose, const nav_msgs::msg::Path & path,
  double tolerance)
{
  if (path.poses.empty()) {
    return false;
  }

  const auto & target = path.poses.back().pose.position;
  return std::sqrt(squaredDistance(current_pose.pose.position, target)) <= tolerance;
}

inline std::size_t findNearestIndex(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const std::vector<geometry_msgs::msg::Point> & goal_points,
  const std::vector<std::size_t> & candidate_indices)
{
  if (candidate_indices.empty()) {
    throw std::runtime_error("candidate index list is empty");
  }

  std::size_t nearest_index = candidate_indices.front();
  double min_distance = std::numeric_limits<double>::max();
  for (const auto candidate_index : candidate_indices) {
    validateIndex(candidate_index, goal_points.size(), "candidate index");
    const auto distance =
      squaredDistance(current_pose.pose.position, goal_points[candidate_index]);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_index = candidate_index;
    }
  }
  return nearest_index;
}

inline std::pair<int, int> computeNextPatrolState(int cursor, int direction, std::size_t count)
{
  if (count <= 1) {
    return {0, 1};
  }

  auto next_cursor = cursor + direction;
  auto next_direction = direction;

  if (next_cursor >= static_cast<int>(count)) {
    next_direction = -1;
    next_cursor = static_cast<int>(count) - 2;
  } else if (next_cursor < 0) {
    next_direction = 1;
    next_cursor = 1;
  }

  return {next_cursor, next_direction};
}

inline bool pathEquivalent(
  const nav_msgs::msg::Path & lhs, const nav_msgs::msg::Path & rhs, double tolerance)
{
  if (lhs.poses.size() != rhs.poses.size()) {
    return false;
  }

  for (std::size_t i = 0; i < lhs.poses.size(); ++i) {
    const auto & lhs_pose = lhs.poses[i].pose.position;
    const auto & rhs_pose = rhs.poses[i].pose.position;
    if (std::sqrt(squaredDistance(lhs_pose, rhs_pose)) > tolerance) {
      return false;
    }
  }

  return true;
}

inline std::vector<std::size_t> toSizeIndices(
  const std::vector<int64_t> & raw_indices, std::size_t total_points, const char * label)
{
  std::vector<std::size_t> indices;
  indices.reserve(raw_indices.size());
  for (const auto raw_index : raw_indices) {
    if (raw_index < 0) {
      throw std::runtime_error(std::string(label) + " contains a negative index");
    }
    indices.push_back(validateIndex(static_cast<std::size_t>(raw_index), total_points, label));
  }
  return indices;
}

}  // namespace pb2025_sentry_behavior::decision

#endif  // PB2025_SENTRY_BEHAVIOR__DECISION_UTILS_HPP_
