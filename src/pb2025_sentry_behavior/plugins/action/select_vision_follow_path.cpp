#include "pb2025_sentry_behavior/plugins/action/select_vision_follow_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
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

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
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

bool isLineTraversable(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end, int occupied_threshold)
{
  const double resolution = static_cast<double>(costmap.info.resolution);
  if (resolution <= 0.0) {
    return true;
  }

  const double distance = planarDistance(start, end);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::msg::Point sample;
    sample.x = start.x + (end.x - start.x) * t;
    sample.y = start.y + (end.y - start.y) * t;
    sample.z = start.z + (end.z - start.z) * t;
    if (!isTraversable(costmap, sample, occupied_threshold)) {
      return false;
    }
  }
  return true;
}

bool hasCircularClearance(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Point & center,
  double clearance_radius, int occupied_threshold)
{
  if (clearance_radius <= 1e-6) {
    return isTraversable(costmap, center, occupied_threshold);
  }

  const double resolution = static_cast<double>(costmap.info.resolution);
  if (resolution <= 0.0) {
    return true;
  }

  const int steps = std::max(1, static_cast<int>(std::ceil(clearance_radius / resolution)));
  for (int ix = -steps; ix <= steps; ++ix) {
    for (int iy = -steps; iy <= steps; ++iy) {
      const double offset_x = static_cast<double>(ix) * resolution;
      const double offset_y = static_cast<double>(iy) * resolution;
      if ((offset_x * offset_x + offset_y * offset_y) > clearance_radius * clearance_radius) {
        continue;
      }

      geometry_msgs::msg::Point sample = center;
      sample.x += offset_x;
      sample.y += offset_y;
      if (!isTraversable(costmap, sample, occupied_threshold)) {
        return false;
      }
    }
  }

  return true;
}

bool hasSegmentClearance(
  const nav_msgs::msg::OccupancyGrid & costmap,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & end,
  double clearance_radius,
  int occupied_threshold)
{
  if (clearance_radius <= 1e-6) {
    return isLineTraversable(costmap, start, end, occupied_threshold);
  }

  const double resolution = static_cast<double>(costmap.info.resolution);
  if (resolution <= 0.0) {
    return true;
  }

  const double distance = planarDistance(start, end);
  const int steps = std::max(1, static_cast<int>(std::ceil(distance / resolution)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    geometry_msgs::msg::Point sample;
    sample.x = start.x + (end.x - start.x) * t;
    sample.y = start.y + (end.y - start.y) * t;
    sample.z = start.z + (end.z - start.z) * t;
    if (!hasCircularClearance(costmap, sample, clearance_radius, occupied_threshold)) {
      return false;
    }
  }

  return true;
}

struct FollowCandidate
{
  geometry_msgs::msg::Point point;
  double score = std::numeric_limits<double>::max();
  double radius = 0.0;
  double border_clearance = 0.0;
};

struct GridNode
{
  int x = 0;
  int y = 0;
  double f = 0.0;
  double g = 0.0;
};

struct GridNodeCompare
{
  bool operator()(const GridNode & lhs, const GridNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

bool worldToGridCell(
  const nav_msgs::msg::OccupancyGrid & costmap,
  const geometry_msgs::msg::Point & point,
  int & mx,
  int & my)
{
  if (costmap.info.width == 0 || costmap.info.height == 0 || costmap.info.resolution <= 0.0) {
    return false;
  }

  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;
  const double resolution = static_cast<double>(costmap.info.resolution);
  mx = static_cast<int>(std::floor((point.x - origin_x) / resolution));
  my = static_cast<int>(std::floor((point.y - origin_y) / resolution));
  return mx >= 0 && my >= 0 &&
    mx < static_cast<int>(costmap.info.width) &&
    my < static_cast<int>(costmap.info.height);
}

geometry_msgs::msg::Point gridCellToWorld(
  const nav_msgs::msg::OccupancyGrid & costmap,
  int mx,
  int my)
{
  geometry_msgs::msg::Point point;
  const double resolution = static_cast<double>(costmap.info.resolution);
  point.x =
    costmap.info.origin.position.x + (static_cast<double>(mx) + 0.5) * resolution;
  point.y =
    costmap.info.origin.position.y + (static_cast<double>(my) + 0.5) * resolution;
  point.z = 0.0;
  return point;
}

std::optional<double> estimateReachablePathLength(
  const nav_msgs::msg::OccupancyGrid & costmap,
  const geometry_msgs::msg::Point & start,
  const geometry_msgs::msg::Point & goal,
  int occupied_threshold,
  double clearance_radius,
  std::size_t max_expansions)
{
  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  if (!worldToGridCell(costmap, start, start_x, start_y) ||
    !worldToGridCell(costmap, goal, goal_x, goal_y))
  {
    return std::nullopt;
  }

  const auto start_world = gridCellToWorld(costmap, start_x, start_y);
  const auto goal_world = gridCellToWorld(costmap, goal_x, goal_y);
  if (!hasCircularClearance(costmap, start_world, clearance_radius, occupied_threshold) ||
    !hasCircularClearance(costmap, goal_world, clearance_radius, occupied_threshold))
  {
    return std::nullopt;
  }

  const int width = static_cast<int>(costmap.info.width);
  const int height = static_cast<int>(costmap.info.height);
  const std::size_t cell_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<bool> closed(cell_count, false);
  auto index_of = [width](int x, int y) -> std::size_t {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
      static_cast<std::size_t>(x);
  };

  auto heuristic = [&costmap](int x0, int y0, int x1, int y1) -> double {
    const double resolution = static_cast<double>(costmap.info.resolution);
    const double dx = static_cast<double>(x1 - x0) * resolution;
    const double dy = static_cast<double>(y1 - y0) * resolution;
    return std::sqrt(dx * dx + dy * dy);
  };

  std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare> open;
  const auto start_index = index_of(start_x, start_y);
  g_score[start_index] = 0.0;
  open.push(GridNode {start_x, start_y, heuristic(start_x, start_y, goal_x, goal_y), 0.0});

  static constexpr std::array<int, 8> kDx{{1, 1, 0, -1, -1, -1, 0, 1}};
  static constexpr std::array<int, 8> kDy{{0, 1, 1, 1, 0, -1, -1, -1}};
  std::size_t expansions = 0;
  while (!open.empty() && expansions < max_expansions) {
    const auto current = open.top();
    open.pop();
    const auto current_index = index_of(current.x, current.y);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;
    ++expansions;

    if (current.x == goal_x && current.y == goal_y) {
      return current.g;
    }

    for (std::size_t dir = 0; dir < kDx.size(); ++dir) {
      const int nx = current.x + kDx[dir];
      const int ny = current.y + kDy[dir];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {
        continue;
      }

      const auto neighbor_index = index_of(nx, ny);
      if (closed[neighbor_index]) {
        continue;
      }

      const auto neighbor_world = gridCellToWorld(costmap, nx, ny);
      if (!hasCircularClearance(costmap, neighbor_world, clearance_radius, occupied_threshold)) {
        continue;
      }

      const double step_cost =
        (kDx[dir] == 0 || kDy[dir] == 0) ? static_cast<double>(costmap.info.resolution) :
        static_cast<double>(costmap.info.resolution) * std::sqrt(2.0);
      const double tentative_g = current.g + step_cost;
      if (tentative_g + 1e-9 >= g_score[neighbor_index]) {
        continue;
      }

      g_score[neighbor_index] = tentative_g;
      open.push(GridNode {
        nx,
        ny,
        tentative_g + heuristic(nx, ny, goal_x, goal_y),
        tentative_g});
    }
  }

  return std::nullopt;
}

double distanceToCostmapBorder(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Point & point)
{
  if (costmap.info.width == 0 || costmap.info.height == 0 || costmap.info.resolution <= 0.0) {
    return std::numeric_limits<double>::max();
  }

  const double origin_x = costmap.info.origin.position.x;
  const double origin_y = costmap.info.origin.position.y;
  const double max_x =
    origin_x + static_cast<double>(costmap.info.width) * costmap.info.resolution;
  const double max_y =
    origin_y + static_cast<double>(costmap.info.height) * costmap.info.resolution;
  return std::min({
    point.x - origin_x,
    max_x - point.x,
    point.y - origin_y,
    max_y - point.y});
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
    if (keepCachedPathIfAllowed("vision target is unavailable on blackboard") ==
      BT::NodeStatus::SUCCESS)
    {
      return BT::NodeStatus::SUCCESS;
    }
    resetCachedPath();
    clearVisualization();
    RCLCPP_DEBUG(logger_, "SelectVisionFollowPath did not receive a vision target");
    return BT::NodeStatus::FAILURE;
  }

  auto current_pose = getInput<geometry_msgs::msg::PoseStamped>("current_pose");

  double pose_timeout_s = 0.5;
  node_->get_parameter("decision.pose.timeout_s", pose_timeout_s);
  bool current_pose_stale = !current_pose;
  if (current_pose) {
    const rclcpp::Time current_pose_stamp(current_pose->header.stamp);
    if (pose_timeout_s > 0.0 && current_pose_stamp.nanoseconds() > 0) {
      const auto age_s = (node_->now() - current_pose_stamp).seconds();
      if (age_s > pose_timeout_s) {
        current_pose_stale = true;
        RCLCPP_WARN_THROTTLE(
          logger_, *node_->get_clock(), 2000,
          "Current pose is stale for vision follow planning: age=%.3fs timeout=%.3fs, keep last valid pose/path if possible",
          age_s, pose_timeout_s);
      } else {
        current_pose_stale = false;
      }
    } else {
      current_pose_stale = false;
    }
  }

  auto costmap = getInput<nav_msgs::msg::OccupancyGrid>("current_costmap");
  const std::string costmap_frame =
    (costmap && !costmap->header.frame_id.empty()) ? costmap->header.frame_id : "";
  geometry_msgs::msg::PoseStamped pose_for_frame;
  if (current_pose) {
    pose_for_frame = *current_pose;
  }
  const auto planning_frame = resolvePlanningFrame(costmap_frame, pose_for_frame, *vision_target);

  const auto target_point = transformTargetPointToFrame(*vision_target, planning_frame);
  if (!target_point) {
    if (has_cached_path_ && last_plan_frame_id_ == planning_frame && !last_path_.poses.empty()) {
      setOutput("path", last_path_);
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "Vision target transform failed, reuse cached vision path in frame '%s'",
        planning_frame.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    if (keepCachedPathIfAllowed("vision target point is invalid or not transformable") ==
      BT::NodeStatus::SUCCESS)
    {
      return BT::NodeStatus::SUCCESS;
    }
    resetCachedPath();
    clearVisualization();
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Vision target_position_map is invalid or not transformable into planning frame '%s'",
      planning_frame.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::optional<geometry_msgs::msg::PoseStamped> transformed_current_pose;
  if (current_pose && !current_pose_stale) {
    transformed_current_pose = transformPoseToFrame(*current_pose, planning_frame);
    if (!transformed_current_pose) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "Failed to transform current pose into planning frame '%s', fall back to cached/derived vision path",
        planning_frame.c_str());
    }
  } else {
    if (current_pose_stale) {
      transformed_current_pose = lookupCurrentPoseFromTf(planning_frame);
      if (transformed_current_pose) {
        RCLCPP_DEBUG_THROTTLE(
          logger_, *node_->get_clock(), 2000,
          "Use TF fallback pose in frame '%s' for vision follow planning",
          planning_frame.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
          logger_, *node_->get_clock(), 2000,
          "Current pose is stale and TF fallback failed for vision follow planning in frame '%s'",
          planning_frame.c_str());
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "Current pose is unavailable for vision follow planning, fall back to cached/derived vision path");
    }
  }

  double attack_radius = 2.0;
  int occupied_threshold = 50;
  int sample_count = 16;
  double follow_arc_half_angle_deg = 90.0;
  double min_replan_interval_s = 0.4;
  double min_goal_shift_m = 0.35;
  double max_goal_angle_step_deg = 18.0;
  double pose_jump_reset_distance_m = 0.8;
  double pose_jump_reset_angle_deg = 55.0;
  double follow_candidate_clearance_radius_m = 0.45;
  double follow_max_path_length_ratio = 1.8;
  int follow_reachability_max_expansions = 5000;
  int follow_reachability_top_candidates = 6;
  node_->get_parameter("decision.vision.attack_radius", attack_radius);
  node_->get_parameter("decision.vision.follow_occupied_threshold", occupied_threshold);
  node_->get_parameter("decision.vision.follow_sample_count", sample_count);
  node_->get_parameter("decision.vision.follow_arc_half_angle_deg", follow_arc_half_angle_deg);
  node_->get_parameter("decision.vision.min_replan_interval_s", min_replan_interval_s);
  node_->get_parameter("decision.vision.min_goal_shift_m", min_goal_shift_m);
  node_->get_parameter(
    "decision.vision.max_goal_angle_step_deg", max_goal_angle_step_deg);
  node_->get_parameter(
    "decision.vision.follow_candidate_clearance_radius_m",
    follow_candidate_clearance_radius_m);
  node_->get_parameter(
    "decision.vision.follow_max_path_length_ratio",
    follow_max_path_length_ratio);
  node_->get_parameter(
    "decision.vision.follow_reachability_max_expansions",
    follow_reachability_max_expansions);
  node_->get_parameter(
    "decision.vision.follow_reachability_top_candidates",
    follow_reachability_top_candidates);
  node_->get_parameter(
    "decision.vision.pose_jump_reset_distance_m", pose_jump_reset_distance_m);
  node_->get_parameter(
    "decision.vision.pose_jump_reset_angle_deg", pose_jump_reset_angle_deg);

  getInput("attack_radius", attack_radius);
  getInput("occupied_threshold", occupied_threshold);
  getInput("sample_count", sample_count);

  attack_radius = std::max(0.1, attack_radius);
  occupied_threshold = std::clamp(occupied_threshold, 1, 100);
  sample_count = std::max(4, sample_count);
  min_replan_interval_s = std::max(0.0, min_replan_interval_s);
  min_goal_shift_m = std::max(0.0, min_goal_shift_m);
  follow_candidate_clearance_radius_m = std::max(0.0, follow_candidate_clearance_radius_m);
  follow_max_path_length_ratio = std::max(1.0, follow_max_path_length_ratio);
  follow_reachability_max_expansions = std::max(200, follow_reachability_max_expansions);
  follow_reachability_top_candidates = std::max(1, follow_reachability_top_candidates);
  pose_jump_reset_distance_m = std::max(0.0, pose_jump_reset_distance_m);
  pose_jump_reset_angle_deg = std::clamp(pose_jump_reset_angle_deg, 0.0, 180.0);
  const double arc_half_angle = std::clamp(
    follow_arc_half_angle_deg * M_PI / 180.0, M_PI / 18.0, M_PI);
  const double max_goal_angle_step_rad = std::clamp(
    max_goal_angle_step_deg * M_PI / 180.0, 0.0, M_PI);
  const double pose_jump_reset_angle_rad = pose_jump_reset_angle_deg * M_PI / 180.0;

  // 当前机器人在目标圆周上的“理想最近角度”。
  // 视觉跟随的原始目标点优先由这个角度决定，而不是继续长期依赖上一拍的包夹侧。
  double nearest_angle = 0.0;
  geometry_msgs::msg::Point current_position = *target_point;
  bool has_current_position = false;
  if (transformed_current_pose) {
    current_position = transformed_current_pose->pose.position;
    has_current_position = true;
  } else if (has_cached_path_ && last_plan_frame_id_ == planning_frame && last_plan_position_) {
    current_position = *last_plan_position_;
    has_current_position = true;
  } else if (has_cached_path_ && last_plan_frame_id_ == planning_frame) {
    current_position = last_selected_goal_;
    has_current_position = true;
  }

  // 这里专门处理“位姿瞬时跳变”场景：
  // 1. loopback 中手动发 /initialpose
  // 2. RViz 2D Pose Estimate 重定位
  // 3. 实车定位链突然校正到新位置
  // 若不清空上一拍缓存，后面的同侧保持和角度平滑会继续拉着目标沿旧侧缓慢过渡，
  // 表面现象就会变成“最近圆周点没有随着当前车位立刻变化”。
  if (has_current_position &&
    shouldResetCachedStateForPoseJump(
      planning_frame, current_position, pose_jump_reset_distance_m))
  {
    const geometry_msgs::msg::Point reference_position =
      last_plan_position_.value_or(last_selected_goal_);
    const double reference_angle = std::atan2(
      reference_position.y - target_point->y, reference_position.x - target_point->x);
    const double current_angle = std::atan2(
      current_position.y - target_point->y, current_position.x - target_point->x);
    const double angle_delta = std::abs(normalizeAngle(current_angle - reference_angle));
    if (angle_delta >= pose_jump_reset_angle_rad) {
      RCLCPP_INFO(
        logger_,
        "Reset cached vision-follow state after pose jump in frame '%s': current=(%.2f, %.2f) last_plan_anchor=(%.2f, %.2f) last_goal=(%.2f, %.2f) dist_threshold=%.2fm angle_delta=%.1fdeg",
        planning_frame.c_str(), current_position.x, current_position.y,
        reference_position.x, reference_position.y, last_selected_goal_.x,
        last_selected_goal_.y, pose_jump_reset_distance_m, angle_delta * 180.0 / M_PI);
      resetCachedPath();
    }
  }

  if (has_current_position && planarDistance(current_position, *target_point) > kPositionEpsilon) {
    nearest_angle = std::atan2(
      current_position.y - target_point->y, current_position.x - target_point->x);
  } else if (
    has_cached_path_ && last_plan_frame_id_ == planning_frame &&
    planarDistance(last_selected_goal_, *target_point) > kPositionEpsilon)
  {
    nearest_angle = std::atan2(
      last_selected_goal_.y - target_point->y, last_selected_goal_.x - target_point->x);
  }

  geometry_msgs::msg::Point nearest_goal_point =
    sampleCirclePoint(*target_point, attack_radius, nearest_angle);
  geometry_msgs::msg::Point selected_point = nearest_goal_point;

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
    const auto fallback_angle_offsets = buildAngleOffsets(sample_count * 2, M_PI);
    const double min_border_clearance =
      std::max(
      std::max(0.25, follow_candidate_clearance_radius_m),
      static_cast<double>(costmap->info.resolution) * 2.0);
    std::vector<FollowCandidate> feasible_candidates;

    auto evaluate_candidates =
      [&](const std::vector<double> & offsets, bool require_line_of_sight) -> bool {
        for (const auto radius_scale : kRadiusScales) {
          const double radius = attack_radius * radius_scale;
          for (const auto angle_offset : offsets) {
            const auto candidate =
              sampleCirclePoint(*target_point, radius, nearest_angle + angle_offset);
            if (!isTraversable(*costmap, candidate, occupied_threshold)) {
              continue;
            }
            if (!hasCircularClearance(
                *costmap, candidate, follow_candidate_clearance_radius_m, occupied_threshold))
            {
              continue;
            }
            const double border_clearance = distanceToCostmapBorder(*costmap, candidate);
            if (border_clearance < min_border_clearance) {
              continue;
            }
            if (
              require_line_of_sight && has_current_position &&
              !hasSegmentClearance(
                *costmap, current_position, candidate,
                follow_candidate_clearance_radius_m, occupied_threshold))
            {
              continue;
            }
            const double candidate_distance_to_robot =
              has_current_position ? planarDistance(candidate, current_position) : 0.0;
            const double candidate_distance_to_ring = planarDistance(candidate, nearest_goal_point);
            const double candidate_border_clearance =
              distanceToCostmapBorder(*costmap, candidate);
            const double candidate_score =
              candidate_distance_to_robot +
              candidate_distance_to_ring * 0.6 -
              candidate_border_clearance * 0.2;
            feasible_candidates.push_back(FollowCandidate {
                candidate, candidate_score, radius, candidate_border_clearance});
          }
        }
        return !feasible_candidates.empty();
      };

    bool found_candidate = evaluate_candidates(angle_offsets, true);
    if (!found_candidate) {
      found_candidate = evaluate_candidates(fallback_angle_offsets, true);
    }
    if (!found_candidate) {
      found_candidate = evaluate_candidates(fallback_angle_offsets, false);
    }

    if (!found_candidate) {
      RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 2000,
        "No free candidate found near the nearest vision-follow ring point, fallback to the raw nearest point");
    } else {
      std::sort(
        feasible_candidates.begin(), feasible_candidates.end(),
        [](const FollowCandidate & lhs, const FollowCandidate & rhs) {
          return lhs.score < rhs.score;
        });

      bool found_reachable_candidate = false;
      const std::size_t candidate_limit = std::min<std::size_t>(
        feasible_candidates.size(),
        static_cast<std::size_t>(follow_reachability_top_candidates));
      if (has_current_position) {
        for (std::size_t i = 0; i < candidate_limit; ++i) {
          const auto & candidate = feasible_candidates[i];
          const double euclidean_distance =
            std::max(planarDistance(current_position, candidate.point), 1e-3);
          const auto reachable_length = estimateReachablePathLength(
            *costmap,
            current_position,
            candidate.point,
            occupied_threshold,
            follow_candidate_clearance_radius_m,
            static_cast<std::size_t>(follow_reachability_max_expansions));
          if (!reachable_length) {
            continue;
          }
          if (*reachable_length <= euclidean_distance * follow_max_path_length_ratio) {
            selected_point = candidate.point;
            found_reachable_candidate = true;
            break;
          }
        }
      }

      if (!found_reachable_candidate) {
        selected_point = feasible_candidates.front().point;
      }
    }
  }

  // 每个决策周期都重新根据当前敌方点和当前车位重新选圆周跟随点，
  // 再用角度限幅做平滑，而不是长期冻结在旧路径上等待阈值触发。
  selected_point = smoothSelectedGoal(
    planning_frame, *target_point, selected_point, max_goal_angle_step_rad);

  bool update_plan_position_anchor = true;
  if (has_cached_path_ && last_plan_frame_id_ == planning_frame && last_plan_time_) {
    const double time_since_last_plan_s = (node_->now() - *last_plan_time_).seconds();
    const double goal_shift_m = planarDistance(selected_point, last_selected_goal_);
    if (time_since_last_plan_s < min_replan_interval_s && goal_shift_m < min_goal_shift_m)
    {
      // 这里只抑制“目标点本身几乎没动”的高频重复下发，
      // 不再要求机器人自身位移超过额外阈值后才允许重选最近圆周点。
      // 这样无论 loopback 手动拖车还是实车自身绕行，
      // 视觉跟随都会持续以“当前车位对应的最近圆周点”为准。
      selected_point = last_selected_goal_;
      update_plan_position_anchor = false;
    }
  }

  const auto now = node_->now();
  publishVisualization(
    planning_frame, current_position, *target_point, nearest_goal_point, selected_point,
    attack_radius, nearest_angle, arc_half_angle, sample_count);

  nav_msgs::msg::Path path;
  path.header.stamp = now;
  path.header.frame_id = planning_frame;
  path.poses.push_back(
    buildFacingPose(selected_point, *target_point, planning_frame, now));
  cachePath(
    *target_point, selected_point, path, planning_frame, now, current_position,
    has_current_position, update_plan_position_anchor, nearest_goal_point);

  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 2000,
    "Vision follow target=(%.2f, %.2f) nearest_ring_goal=(%.2f, %.2f) selected_goal=(%.2f, %.2f) attack_radius=%.2f frame=%s",
    target_point->x, target_point->y, nearest_goal_point.x, nearest_goal_point.y, selected_point.x,
    selected_point.y, attack_radius, planning_frame.c_str());

  setOutput("path", path);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SelectVisionFollowPathAction::keepCachedPathIfAllowed(const char * reason)
{
  double override_hold_s = 0.0;
  node_->get_parameter("decision.vision.override_hold_s", override_hold_s);

  if (
    override_hold_s > 0.0 && has_cached_path_ && last_plan_time_ && !last_path_.poses.empty() &&
    (node_->now() - *last_plan_time_).seconds() <= override_hold_s)
  {
    setOutput("path", last_path_);
    RCLCPP_DEBUG_THROTTLE(
      logger_, *node_->get_clock(), 1000,
      "Keep cached vision-follow path for %.2fs after transient invalid sample: %s",
      override_hold_s, reason);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
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

std::optional<geometry_msgs::msg::PoseStamped>
SelectVisionFollowPathAction::lookupCurrentPoseFromTf(const std::string & target_frame) const
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

geometry_msgs::msg::Point SelectVisionFollowPathAction::smoothSelectedGoal(
  const std::string & planning_frame, const geometry_msgs::msg::Point & target_point,
  const geometry_msgs::msg::Point & raw_selected_goal, double max_goal_angle_step_rad) const
{
  if (
    !has_cached_path_ || last_plan_frame_id_ != planning_frame || last_path_.poses.empty() ||
    max_goal_angle_step_rad <= kPositionEpsilon)
  {
    return raw_selected_goal;
  }

  const double last_radius = planarDistance(last_selected_goal_, target_point);
  const double raw_radius = planarDistance(raw_selected_goal, target_point);
  if (last_radius <= kPositionEpsilon || raw_radius <= kPositionEpsilon) {
    return raw_selected_goal;
  }

  const double last_angle = std::atan2(
    last_selected_goal_.y - target_point.y, last_selected_goal_.x - target_point.x);
  const double raw_angle = std::atan2(
    raw_selected_goal.y - target_point.y, raw_selected_goal.x - target_point.x);
  const double angle_delta = normalizeAngle(raw_angle - last_angle);
  if (std::abs(angle_delta) <= max_goal_angle_step_rad) {
    return raw_selected_goal;
  }

  const double limited_angle =
    last_angle + std::copysign(max_goal_angle_step_rad, angle_delta);
  return sampleCirclePoint(target_point, raw_radius, limited_angle);
}

bool SelectVisionFollowPathAction::shouldResetCachedStateForPoseJump(
  const std::string & planning_frame, const geometry_msgs::msg::Point & current_position,
  double pose_jump_reset_distance_m) const
{
  if (
    !has_cached_path_ || last_plan_frame_id_ != planning_frame ||
    pose_jump_reset_distance_m <= kPositionEpsilon)
  {
    return false;
  }

  // 优先拿“上一次真正下发目标时的机器人位姿”做参考；
  // 若当时没有可靠当前位姿，再退化到上一拍已选跟随点。
  const geometry_msgs::msg::Point reference_position =
    last_plan_position_.value_or(last_selected_goal_);
  return planarDistance(current_position, reference_position) >= pose_jump_reset_distance_m;
}

void SelectVisionFollowPathAction::cachePath(
  const geometry_msgs::msg::Point & target_point,
  const geometry_msgs::msg::Point & selected_goal, const nav_msgs::msg::Path & path,
  const std::string & planning_frame, const rclcpp::Time & now,
  const geometry_msgs::msg::Point & current_position, bool has_current_position,
  bool update_plan_position_anchor, const geometry_msgs::msg::Point & nearest_goal_point)
{
  last_target_point_ = target_point;
  last_nearest_goal_point_ = nearest_goal_point;
  last_selected_goal_ = selected_goal;
  if (update_plan_position_anchor) {
    if (has_current_position) {
      last_plan_position_ = current_position;
    } else {
      last_plan_position_.reset();
    }
  }
  last_path_ = path;
  last_plan_time_ = now;
  last_plan_frame_id_ = planning_frame;
  has_cached_path_ = true;
}

void SelectVisionFollowPathAction::publishVisualization(
  const std::string & planning_frame, const geometry_msgs::msg::Point & current_position,
  const geometry_msgs::msg::Point & target_point,
  const geometry_msgs::msg::Point & nearest_goal_point,
  const geometry_msgs::msg::Point & selected_goal, double attack_radius, double nearest_angle,
  double arc_half_angle, int sample_count)
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
    target_point, attack_radius, nearest_angle, arc_half_angle, std::max(24, sample_count));
  markers.markers.push_back(arc_marker);

  visualization_msgs::msg::Marker nearest_goal_marker;
  nearest_goal_marker.header.frame_id = planning_frame;
  nearest_goal_marker.header.stamp = stamp;
  nearest_goal_marker.ns = "vision_follow";
  nearest_goal_marker.id = 2;
  nearest_goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  nearest_goal_marker.action = visualization_msgs::msg::Marker::ADD;
  nearest_goal_marker.pose.orientation.w = 1.0;
  nearest_goal_marker.pose.position = nearest_goal_point;
  nearest_goal_marker.scale.x = 0.20;
  nearest_goal_marker.scale.y = 0.20;
  nearest_goal_marker.scale.z = 0.20;
  nearest_goal_marker.color.r = 0.15F;
  nearest_goal_marker.color.g = 0.55F;
  nearest_goal_marker.color.b = 1.0F;
  nearest_goal_marker.color.a = 0.98F;
  markers.markers.push_back(nearest_goal_marker);

  visualization_msgs::msg::Marker selected_goal_marker;
  selected_goal_marker.header.frame_id = planning_frame;
  selected_goal_marker.header.stamp = stamp;
  selected_goal_marker.ns = "vision_follow";
  selected_goal_marker.id = 3;
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
  path_marker.id = 4;
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
  last_plan_position_.reset();
  last_plan_time_.reset();
  last_plan_frame_id_.clear();
  last_path_ = nav_msgs::msg::Path{};
  last_target_point_ = geometry_msgs::msg::Point{};
  last_nearest_goal_point_ = geometry_msgs::msg::Point{};
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
