// Copyright 2024 Polaris Xia
 

#include "ats_nav2_plugins/behaviors/back_up_free_space.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ats_nav2_behaviors
{

namespace
{

constexpr unsigned char kInscribedObstacleCost = 253;

// 统一角度到 [-pi, pi]。
// 恢复方向搜索和“上一条方向黏性”都依赖稳定的角度差计算。
double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

const char * planSourceName(ats_nav2_behaviors::BackUpFreeSpace::PlanSource source)
{
  switch (source) {
    case ats_nav2_behaviors::BackUpFreeSpace::PlanSource::CORRIDOR_PRIMARY:
      return "corridor";
    case ats_nav2_behaviors::BackUpFreeSpace::PlanSource::CENTROID_FALLBACK:
      return "centroid_fallback";
    default:
      return "unknown";
  }
}

}  // namespace

void BackUpFreeSpace::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(node, "global_frame", rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, "max_radius", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "service_name", rclcpp::ParameterValue("local_costmap/get_costmap"));
  nav2_util::declare_parameter_if_not_declared(
    node, "max_allowed_cost", rclcpp::ParameterValue(96));
  nav2_util::declare_parameter_if_not_declared(node, "visualize", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node, "search_half_span_deg", rclcpp::ParameterValue(140.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "search_angle_increment_deg", rclcpp::ParameterValue(10.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "trajectory_sample_step", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, "near_sample_step", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, "far_sample_step", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, "layered_sampling_split_distance", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(
    node, "corridor_half_width", rclcpp::ParameterValue(0.22));
  nav2_util::declare_parameter_if_not_declared(
    node, "corridor_lateral_step", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, "far_corridor_lateral_step", rclcpp::ParameterValue(0.12));
  nav2_util::declare_parameter_if_not_declared(
    node, "minimum_release_distance", rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(
    node, "dynamic_obstacle_prediction_enabled", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node, "prediction_horizon_s", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(
    node, "prefix_velocity_alpha", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, "predictive_block_margin", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, "heading_stickiness_weight", rclcpp::ParameterValue(6.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "replanning_cooldown_s", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, "blocked_enter_cycles", rclcpp::ParameterValue(3));
  nav2_util::declare_parameter_if_not_declared(
    node, "clear_exit_cycles", rclcpp::ParameterValue(2));
  nav2_util::declare_parameter_if_not_declared(
    node, "max_replan_attempts", rclcpp::ParameterValue(6));
  nav2_util::declare_parameter_if_not_declared(
    node, "speed_filter_tau", rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(
    node, "translational_acc_limit", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, "translational_decel_limit", rclcpp::ParameterValue(1.2));
  nav2_util::declare_parameter_if_not_declared(
    node, "minimum_speed_xy", rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
    node, "high_cost_speed_threshold", rclcpp::ParameterValue(48.0));
  nav2_util::declare_parameter_if_not_declared(
    node, "high_cost_speed_min_scale", rclcpp::ParameterValue(0.55));
  nav2_util::declare_parameter_if_not_declared(
    node, "goal_tolerance", rclcpp::ParameterValue(0.04));
  nav2_util::declare_parameter_if_not_declared(
    node, "monitor_lookahead_distance", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, "enable_full_circle_fallback", rclcpp::ParameterValue(true));

  // 下面这组参数可以分成四类理解：
  // 1. 方向搜索：search_half_span_deg / search_angle_increment_deg
  // 2. 轨迹批量检测：trajectory_sample_step / corridor_half_width / corridor_lateral_step
  // 3. 状态滞回：blocked_enter_cycles / clear_exit_cycles / replanning_cooldown_s
  // 4. 速度平滑：speed_filter_tau / translational_acc_limit / translational_decel_limit
  node->get_parameter("global_frame", global_frame_);
  node->get_parameter("max_radius", max_radius_);
  node->get_parameter("service_name", service_name_);
  node->get_parameter("max_allowed_cost", max_allowed_cost_);
  node->get_parameter("visualize", visualize_);
  node->get_parameter("search_half_span_deg", search_half_span_deg_);
  node->get_parameter("search_angle_increment_deg", search_angle_increment_deg_);
  node->get_parameter("trajectory_sample_step", trajectory_sample_step_);
  node->get_parameter("near_sample_step", near_sample_step_);
  node->get_parameter("far_sample_step", far_sample_step_);
  node->get_parameter("layered_sampling_split_distance", layered_sampling_split_distance_);
  node->get_parameter("corridor_half_width", corridor_half_width_);
  node->get_parameter("corridor_lateral_step", corridor_lateral_step_);
  node->get_parameter("far_corridor_lateral_step", far_corridor_lateral_step_);
  node->get_parameter("minimum_release_distance", minimum_release_distance_);
  node->get_parameter(
    "dynamic_obstacle_prediction_enabled", dynamic_obstacle_prediction_enabled_);
  node->get_parameter("prediction_horizon_s", prediction_horizon_s_);
  node->get_parameter("prefix_velocity_alpha", prefix_velocity_alpha_);
  node->get_parameter("predictive_block_margin", predictive_block_margin_);
  node->get_parameter("heading_stickiness_weight", heading_stickiness_weight_);
  node->get_parameter("replanning_cooldown_s", replanning_cooldown_s_);
  node->get_parameter("blocked_enter_cycles", blocked_enter_cycles_);
  node->get_parameter("clear_exit_cycles", clear_exit_cycles_);
  node->get_parameter("max_replan_attempts", max_replan_attempts_);
  node->get_parameter("speed_filter_tau", speed_filter_tau_);
  node->get_parameter("translational_acc_limit", translational_acc_limit_);
  node->get_parameter("translational_decel_limit", translational_decel_limit_);
  node->get_parameter("minimum_speed_xy", minimum_speed_xy_);
  node->get_parameter("high_cost_speed_threshold", high_cost_speed_threshold_);
  node->get_parameter("high_cost_speed_min_scale", high_cost_speed_min_scale_);
  node->get_parameter("goal_tolerance", goal_tolerance_);
  node->get_parameter("monitor_lookahead_distance", monitor_lookahead_distance_);
  node->get_parameter("enable_full_circle_fallback", enable_full_circle_fallback_);

  costmap_client_ = node->create_client<nav2_msgs::srv::GetCostmap>(service_name_);

  if (visualize_) {
    marker_pub_ = node->template create_publisher<visualization_msgs::msg::MarkerArray>(
      "back_up_free_space_markers", 1);
    marker_pub_->on_activate();
  }
}

void BackUpFreeSpace::onCleanup()
{
  costmap_client_.reset();
  marker_pub_.reset();
  resetExecutionState();
}

nav2_behaviors::Status BackUpFreeSpace::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (!nav2_util::getCurrentPose(
        initial_pose_, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
    RCLCPP_ERROR(logger_, "Initial robot pose is not available.");
    return nav2_behaviors::Status::FAILED;
  }

  nav2_msgs::msg::Costmap costmap;
  if (!fetchCostmap(costmap)) {
    return nav2_behaviors::Status::FAILED;
  }

  command_distance_abs_ = std::min(max_radius_, std::fabs(command->target.x));
  command_speed_abs_ = std::fabs(command->speed);
  command_x_ = command->target.x;
  command_time_allowance_ = command->time_allowance;
  end_time_ = clock_->now() + command_time_allowance_;
  plan_start_pose_ = initial_pose_;
  completed_distance_before_plan_ = 0.0;
  last_safe_prefix_distance_.reset();
  estimated_prefix_rate_ = 0.0;
  filtered_cmd_ = geometry_msgs::msg::Twist {};
  blocked_cycles_ = 0;
  clear_cycles_ = 0;
  failed_replan_attempts_ = 0;
  last_cycle_time_ = clock_->now();
  last_replan_time_ = clock_->now();
  execution_state_ = RecoveryExecutionState::PLANNING;

  const auto current_pose_2d = poseToPose2D(initial_pose_);
  // 这里不再像旧实现那样只找一个“最空方向”然后直接开退，
  // 而是先规划一条短时恢复轨迹，后续执行和重规划都围绕这条轨迹展开。
  if (!planEscapeTrajectory(costmap, current_pose_2d, command_distance_abs_, active_plan_)) {
    if (!planCentroidFallbackTrajectory(
          costmap, current_pose_2d, command_distance_abs_, active_plan_))
    {
      RCLCPP_WARN(
        logger_,
        "No smooth omni recovery trajectory found within %.2fm around the robot.",
        command_distance_abs_);
      return nav2_behaviors::Status::FAILED;
    }
    active_plan_source_ = PlanSource::CENTROID_FALLBACK;
    RCLCPP_WARN(
      logger_,
      "Fallback to centroid-based recovery heading %.2fdeg.",
      active_plan_.heading * 180.0 / M_PI);
  } else {
    active_plan_source_ = PlanSource::CORRIDOR_PRIMARY;
  }

  active_plan_average_cost_ = computePlanAverageCost(costmap, active_plan_);
  previous_plan_heading_ = active_plan_.heading;
  has_previous_plan_heading_ = true;
  execution_state_ = RecoveryExecutionState::EXECUTING;
  if (visualize_) {
    visualizePlan(current_pose_2d, active_plan_);
  }
  RCLCPP_WARN(
    logger_,
    "Start omni recovery: source=%s distance=%.2f heading=%.2fdeg score=%.2f avg_cost=%.2f",
    planSourceName(active_plan_source_),
    active_plan_.distance, active_plan_.heading * 180.0 / M_PI, active_plan_.score,
    active_plan_average_cost_);

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status BackUpFreeSpace::onCycleUpdate()
{
  rclcpp::Duration time_remaining = end_time_ - clock_->now();
  if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "Exceeded time allowance before reaching the "
      "DriveOnHeading goal - Exiting DriveOnHeading");
    return nav2_behaviors::Status::FAILED;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
        current_pose, *tf_, global_frame_, robot_base_frame_, transform_tolerance_)) {
      RCLCPP_ERROR(logger_, "Current robot pose is not available.");
      return nav2_behaviors::Status::FAILED;
  }

  const auto now = clock_->now();
  const double dt =
    last_cycle_time_ ? std::max(1e-3, (now - *last_cycle_time_).seconds()) : 1.0 / cycle_frequency_;
  last_cycle_time_ = now;

  const auto current_pose_2d = poseToPose2D(current_pose);
  const double segment_progress = computeSegmentProgress(current_pose_2d);
  const double total_distance_traveled = completed_distance_before_plan_ + segment_progress;
  const double remaining_distance = std::max(0.0, command_distance_abs_ - total_distance_traveled);

  feedback_->distance_traveled = total_distance_traveled;
  action_server_->publish_feedback(feedback_);

  if (remaining_distance <= goal_tolerance_) {
    stopRobot();
    return nav2_behaviors::Status::SUCCEEDED;
  }

  // 关键优化 1：
  // 不再采用“移动一个很小步长后立刻判一次”的离散方式，
  // 而是对当前恢复轨迹前向一段 lookahead 做批量采样检测。
  // 这样既能更早发现动态遮挡，也能避免每拍都因为单个采样点抖动而切状态。
  const double safe_prefix_distance = computeSafePrefixDistance(current_pose_2d, remaining_distance);
  bool trajectory_prefix_safe = safe_prefix_distance >= std::min(monitor_lookahead_distance_, remaining_distance);

  // 第二阶段优化：
  // 不只看“当前这一拍前方还能不能走”，还估计这个安全前缀是否在快速缩短。
  // 如果前沿正在以较快速度向机器人逼近，就说明多半是动态障碍重新挡住了恢复走廊，
  // 这时提前进入 BLOCKED / 重规划，比等到真正碰上再急停更平滑。
  if (last_safe_prefix_distance_ && dt > 1e-3) {
    const double raw_prefix_rate = (safe_prefix_distance - *last_safe_prefix_distance_) / dt;
    const double alpha = std::clamp(prefix_velocity_alpha_, 0.0, 1.0);
    estimated_prefix_rate_ =
      alpha * raw_prefix_rate + (1.0 - alpha) * estimated_prefix_rate_;
  } else {
    estimated_prefix_rate_ = 0.0;
  }
  last_safe_prefix_distance_ = safe_prefix_distance;

  if (dynamic_obstacle_prediction_enabled_) {
    const double predicted_safe_prefix =
      safe_prefix_distance + estimated_prefix_rate_ * std::max(0.0, prediction_horizon_s_);
    const double required_prefix =
      std::min(monitor_lookahead_distance_, remaining_distance) - predictive_block_margin_;
    if (predicted_safe_prefix < required_prefix) {
      trajectory_prefix_safe = false;
      RCLCPP_DEBUG_THROTTLE(
        logger_, *clock_, 1000,
        "Predictive recovery block: current_prefix=%.2f predicted_prefix=%.2f prefix_rate=%.2f required_prefix=%.2f",
        safe_prefix_distance, predicted_safe_prefix, estimated_prefix_rate_, required_prefix);
    }
  }

  if (trajectory_prefix_safe) {
    clear_cycles_++;
    blocked_cycles_ = 0;
  } else {
    blocked_cycles_++;
    clear_cycles_ = 0;
  }

  if (
    execution_state_ == RecoveryExecutionState::EXECUTING &&
    blocked_cycles_ >= blocked_enter_cycles_)
  {
    execution_state_ = RecoveryExecutionState::BLOCKED;
    RCLCPP_WARN(
      logger_,
      "Recovery trajectory blocked for %d consecutive cycles, enter BLOCKED state.",
      blocked_cycles_);
  } else if (
    execution_state_ == RecoveryExecutionState::BLOCKED &&
    clear_cycles_ >= clear_exit_cycles_)
  {
    execution_state_ = RecoveryExecutionState::EXECUTING;
    RCLCPP_INFO(
      logger_,
      "Recovery trajectory stayed clear for %d cycles, exit BLOCKED state.",
      clear_cycles_);
  }

  if (
    execution_state_ == RecoveryExecutionState::BLOCKED && last_replan_time_ &&
    (now - *last_replan_time_).seconds() >= replanning_cooldown_s_)
  {
    // 关键优化 2：
    // 使用“连续阻塞计数 + 重规划冷却时间”形成状态滞回，
    // 避免 normal / avoid / recovery 在障碍边界附近每拍反复横跳。
    if (replanFromCurrentPose(current_pose, remaining_distance, total_distance_traveled)) {
      blocked_cycles_ = 0;
      clear_cycles_ = 0;
      failed_replan_attempts_ = 0;
      execution_state_ = RecoveryExecutionState::EXECUTING;
      if (visualize_) {
        visualizePlan(current_pose_2d, active_plan_);
      }
    } else {
      failed_replan_attempts_++;
      last_replan_time_ = now;
      RCLCPP_WARN(
        logger_,
        "Recovery replan failed %d/%d times.",
        failed_replan_attempts_, max_replan_attempts_);
      if (failed_replan_attempts_ >= max_replan_attempts_) {
        stopRobot();
        RCLCPP_WARN(logger_, "Recovery failed after repeated replanning attempts.");
        return nav2_behaviors::Status::FAILED;
      }
    }
  }

  geometry_msgs::msg::Twist desired_cmd;
  if (execution_state_ == RecoveryExecutionState::EXECUTING) {
    desired_cmd = buildDesiredCommand(remaining_distance);
  } else {
    desired_cmd = geometry_msgs::msg::Twist {};
  }

  // 关键优化 3：
  // 对恢复速度做一阶低通 + 加减速度限幅。
  // 全向舵轮底盘在恢复链里若直接阶跃切到目标速度，最容易出现电机抖动和“挫一下”的卡顿感。
  const auto filtered_cmd = smoothCommand(desired_cmd, dt);
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(filtered_cmd);
  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

bool BackUpFreeSpace::fetchCostmap(nav2_msgs::msg::Costmap & costmap)
{
  // 恢复行为使用服务获取一份瞬时 costmap 快照：
  // 这样每次 onRun / replan 都能基于最新障碍状态重新做局部规划。
  while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger_, "Interrupted while waiting for the costmap service.");
      return false;
    }
    RCLCPP_WARN(logger_, "Recovery costmap service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
  auto result = costmap_client_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::timeout) {
    RCLCPP_ERROR(logger_, "Timeout while waiting for recovery costmap response.");
    return false;
  }

  costmap = result.get()->map;
  return true;
}

geometry_msgs::msg::Pose2D BackUpFreeSpace::poseToPose2D(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  geometry_msgs::msg::Pose2D pose_2d;
  pose_2d.x = pose.pose.position.x;
  pose_2d.y = pose.pose.position.y;
  pose_2d.theta = tf2::getYaw(pose.pose.orientation);
  return pose_2d;
}

bool BackUpFreeSpace::planEscapeTrajectory(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  double target_distance, EscapePlan & best_plan)
{
  best_plan = EscapePlan();
  best_plan.score = std::numeric_limits<double>::infinity();

  const double search_half_span = std::clamp(search_half_span_deg_ * M_PI / 180.0, 0.0, M_PI);
  const double angle_increment = std::max(1e-3, search_angle_increment_deg_ * M_PI / 180.0);
  const double rear_heading = normalizeAngle(pose.theta + M_PI);

  // 以“车尾方向”为主搜索轴，而不是硬编码只允许 x 轴倒车。
  // 对全向舵轮这点非常重要，因为斜后退 / 侧后退往往比纯后退更容易脱困。
  auto search_range = [&](double start, double end) {
    for (double angle = start; angle <= end + 1e-6; angle += angle_increment) {
      EscapePlan candidate;
      if (!evaluateCandidateTrajectory(
            costmap, pose, normalizeAngle(angle), target_distance, candidate))
      {
        continue;
      }
      if (!best_plan.valid || candidate.score < best_plan.score) {
        best_plan = candidate;
      }
    }
  };

  search_range(rear_heading - search_half_span, rear_heading + search_half_span);

  // 对全向舵轮底盘保留一个兜底：
  // 如果后向和两侧都没有可退让轨迹，再放开到全角域搜索，优先保证能脱困。
  if (!best_plan.valid && enable_full_circle_fallback_) {
    search_range(-M_PI, M_PI);
  }

  return best_plan.valid;
}

bool BackUpFreeSpace::planCentroidFallbackTrajectory(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  double target_distance, EscapePlan & fallback_plan) const
{
  fallback_plan = EscapePlan();

  const double resolution = static_cast<double>(costmap.metadata.resolution);
  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  const double radius_limit = std::max(target_distance, resolution);

  double accum_x = 0.0;
  double accum_y = 0.0;
  int free_count = 0;

  for (int mx = 0; mx < size_x; ++mx) {
    for (int my = 0; my < size_y; ++my) {
      const auto index =
        static_cast<std::size_t>(my) * static_cast<std::size_t>(size_x) +
        static_cast<std::size_t>(mx);
      if (index >= costmap.data.size()) {
        continue;
      }

      const double x = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
      const double y = origin_y + (static_cast<double>(my) + 0.5) * resolution;
      const double distance = std::hypot(x - pose.x, y - pose.y);
      if (distance > radius_limit) {
        continue;
      }

      const unsigned char cost = static_cast<unsigned char>(costmap.data[index]);
      if (cost >= kInscribedObstacleCost || static_cast<int>(cost) > max_allowed_cost_) {
        continue;
      }

      accum_x += x;
      accum_y += y;
      free_count++;
    }
  }

  if (free_count < 5) {
    return false;
  }

  const double centroid_x = accum_x / static_cast<double>(free_count);
  const double centroid_y = accum_y / static_cast<double>(free_count);
  const double heading = std::atan2(centroid_y - pose.y, centroid_x - pose.x);

  EscapePlan candidate;
  if (!evaluateCandidateTrajectory(costmap, pose, heading, target_distance, candidate)) {
    return false;
  }

  fallback_plan = candidate;
  return true;
}

bool BackUpFreeSpace::evaluateCandidateTrajectory(
  const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
  double heading, double target_distance, EscapePlan & candidate) const
{
  candidate = EscapePlan();
  candidate.heading = heading;
  candidate.distance = target_distance;
  candidate.score = std::numeric_limits<double>::infinity();

  const double resolution = static_cast<double>(costmap.metadata.resolution);
  const double near_sample_step = std::max(
    near_sample_step_ > 0.0 ? near_sample_step_ : trajectory_sample_step_, resolution);
  const double far_sample_step = std::max(
    far_sample_step_ > 0.0 ? far_sample_step_ : trajectory_sample_step_, near_sample_step);
  const double split_distance = std::max(layered_sampling_split_distance_, near_sample_step);
  const double near_lateral_step = std::max(corridor_lateral_step_, resolution);
  const double far_lateral_step = std::max(far_corridor_lateral_step_, near_lateral_step);
  const double nx = -std::sin(heading);
  const double ny = std::cos(heading);
  double accumulated_cost = 0.0;
  int sampled_cells = 0;
  double safe_distance = 0.0;

  // 对每个候选方向，不是只检查一条“中心线”，
  // 而是构造一条带宽度的恢复走廊并做批量采样。
  // 这能显著减少因为单格点抖动引起的“能走/不能走”来回切换。
  for (double s = near_sample_step; s <= target_distance + 1e-6;) {
    const bool use_near_sampling = s <= split_distance;
    const double sample_step = use_near_sampling ? near_sample_step : far_sample_step;
    const double lateral_step = use_near_sampling ? near_lateral_step : far_lateral_step;
    geometry_msgs::msg::Point center_point;
    center_point.x = pose.x + s * std::cos(heading);
    center_point.y = pose.y + s * std::sin(heading);
    center_point.z = 0.0;

    bool current_ring_safe = true;
    for (
      double offset = -corridor_half_width_; offset <= corridor_half_width_ + 1e-6;
      offset += lateral_step)
    {
      const double sample_x = center_point.x + nx * offset;
      const double sample_y = center_point.y + ny * offset;
      const auto cost = sampleCost(costmap, sample_x, sample_y);
      if (!cost.has_value()) {
        current_ring_safe = false;
        break;
      }
      if (*cost >= kInscribedObstacleCost || static_cast<int>(*cost) > max_allowed_cost_) {
        current_ring_safe = false;
        break;
      }
      accumulated_cost += static_cast<double>(*cost);
      sampled_cells++;
    }

    if (!current_ring_safe) {
      break;
    }

    candidate.centerline.push_back(center_point);
    safe_distance = s;
    s += sample_step;
  }

  if (safe_distance < minimum_release_distance_ || candidate.centerline.empty() || sampled_cells == 0) {
    return false;
  }

  // 第一阶段优化：分段放行
  // 如果整条恢复目标距离不全通，不直接判整条轨迹失败，
  // 而是允许先释放当前连续可通的最远安全段。
  // 这样可以避免底盘“明明前面 30cm 是安全的，却因为 80cm 外有障碍就原地卡死”。
  candidate.distance = std::min(target_distance, safe_distance);
  candidate.goal_point = candidate.centerline.back();
  candidate.valid = true;

  // 评分包含三部分：
  // 1. average_cost：平均代价越低越好，代表恢复过程中离障碍更远
  // 2. rear_bias：越接近车尾主后退方向越好，减少不必要的大侧移
  // 3. heading_stickiness：与上一条恢复方向越接近越好，减少左右抽动
  const double average_cost = accumulated_cost / static_cast<double>(sampled_cells);
  const double rear_heading = normalizeAngle(pose.theta + M_PI);
  const double rear_bias = std::abs(normalizeAngle(heading - rear_heading));
  const double heading_stickiness =
    has_previous_plan_heading_ ? std::abs(normalizeAngle(heading - previous_plan_heading_)) : 0.0;
  candidate.score = average_cost + rear_bias * 8.0 + heading_stickiness * heading_stickiness_weight_;
  return true;
}

std::optional<unsigned char> BackUpFreeSpace::sampleCost(
  const nav2_msgs::msg::Costmap & costmap, double x, double y) const
{
  const double resolution = static_cast<double>(costmap.metadata.resolution);
  const double origin_x = costmap.metadata.origin.position.x;
  const double origin_y = costmap.metadata.origin.position.y;
  const int size_x = static_cast<int>(costmap.metadata.size_x);
  const int size_y = static_cast<int>(costmap.metadata.size_y);
  const int map_x = static_cast<int>(std::floor((x - origin_x) / resolution));
  const int map_y = static_cast<int>(std::floor((y - origin_y) / resolution));

  if (map_x < 0 || map_x >= size_x || map_y < 0 || map_y >= size_y) {
    return std::nullopt;
  }

  const auto index =
    static_cast<std::size_t>(map_y) * static_cast<std::size_t>(size_x) +
    static_cast<std::size_t>(map_x);
  if (index >= costmap.data.size()) {
    return std::nullopt;
  }

  return static_cast<unsigned char>(costmap.data[index]);
}

double BackUpFreeSpace::computePlanAverageCost(
  const nav2_msgs::msg::Costmap & costmap, const EscapePlan & plan) const
{
  if (!plan.valid || plan.centerline.empty()) {
    return 0.0;
  }

  double accumulated_cost = 0.0;
  int sampled_points = 0;
  for (const auto & point : plan.centerline) {
    const auto cost = sampleCost(costmap, point.x, point.y);
    if (!cost.has_value()) {
      continue;
    }
    accumulated_cost += static_cast<double>(*cost);
    sampled_points++;
  }

  return sampled_points > 0 ? accumulated_cost / static_cast<double>(sampled_points) : 0.0;
}

bool BackUpFreeSpace::isTrajectoryPrefixSafe(
  const geometry_msgs::msg::Pose2D & pose, double remaining_distance)
{
  const double safe_prefix_distance = computeSafePrefixDistance(pose, remaining_distance);
  return safe_prefix_distance >= std::min(monitor_lookahead_distance_, remaining_distance);
}

double BackUpFreeSpace::computeSafePrefixDistance(
  const geometry_msgs::msg::Pose2D & pose, double remaining_distance) const
{
  if (!active_plan_.valid) {
    return 0.0;
  }

  const double check_distance = std::min(monitor_lookahead_distance_, remaining_distance);
  const double step = std::max(near_sample_step_ > 0.0 ? near_sample_step_ : trajectory_sample_step_, 0.03);
  bool fetch_data = true;
  double safe_prefix_distance = 0.0;

  // 这里只监控“当前轨迹前方一小段距离”的安全性，而不是整条剩余路径都重新扫一遍。
  // 原因：
  // 1. 对动态障碍，当前最重要的是眼前一小段是否还能继续走
  // 2. 这能把检测做成一个连续的 lookahead 过程，减少旧版那种“走一点停一下”的离散感
  for (double s = step; s <= check_distance + 1e-6; s += step) {
    geometry_msgs::msg::Pose2D sample_pose = pose;
    sample_pose.x += s * std::cos(active_plan_.heading);
    sample_pose.y += s * std::sin(active_plan_.heading);
    if (!collision_checker_->isCollisionFree(sample_pose, fetch_data)) {
      return safe_prefix_distance;
    }
    fetch_data = false;
    safe_prefix_distance = s;
  }

  return safe_prefix_distance;
}

double BackUpFreeSpace::computeSegmentProgress(const geometry_msgs::msg::Pose2D & pose) const
{
  if (!active_plan_.valid) {
    return 0.0;
  }

  const double dx = pose.x - plan_start_pose_.pose.position.x;
  const double dy = pose.y - plan_start_pose_.pose.position.y;
  const double projected =
    dx * std::cos(active_plan_.heading) + dy * std::sin(active_plan_.heading);
  return std::clamp(projected, 0.0, active_plan_.distance);
}

geometry_msgs::msg::Twist BackUpFreeSpace::buildDesiredCommand(double remaining_distance) const
{
  geometry_msgs::msg::Twist desired_cmd;
  if (!active_plan_.valid || remaining_distance <= goal_tolerance_) {
    return desired_cmd;
  }

  // 根据剩余距离反推“还能以多大速度安全停住”。
  // 这样恢复末段会自然减速，而不是等快到目标时突然急刹。
  const double stop_speed =
    std::sqrt(std::max(0.0, 2.0 * translational_decel_limit_ * remaining_distance));
  double target_speed = std::min(command_speed_abs_, stop_speed);

  const double high_cost_threshold = std::max(1.0, high_cost_speed_threshold_);
  const double min_cost_scale = std::clamp(high_cost_speed_min_scale_, 0.1, 1.0);
  if (active_plan_average_cost_ > high_cost_threshold) {
    const double overload =
      std::min(1.0, (active_plan_average_cost_ - high_cost_threshold) /
      std::max(1.0, 252.0 - high_cost_threshold));
    const double cost_scale = 1.0 - overload * (1.0 - min_cost_scale);
    target_speed *= cost_scale;
  }

  if (target_speed < minimum_speed_xy_ && remaining_distance > goal_tolerance_) {
    target_speed = minimum_speed_xy_;
  }

  desired_cmd.linear.x = std::cos(active_plan_.heading) * target_speed;
  desired_cmd.linear.y = std::sin(active_plan_.heading) * target_speed;
  desired_cmd.angular.z = 0.0;
  return desired_cmd;
}

geometry_msgs::msg::Twist BackUpFreeSpace::smoothCommand(
  const geometry_msgs::msg::Twist & desired_cmd, double dt)
{
  geometry_msgs::msg::Twist smoothed_cmd = filtered_cmd_;
  const double alpha = dt / (speed_filter_tau_ + dt);
  const double accel_limit = std::max(1e-3, translational_acc_limit_) * dt;
  const double decel_limit = std::max(1e-3, translational_decel_limit_) * dt;

  // 每个平移轴都做两步：
  // 1. 先一阶低通，吸收目标速度的尖跳
  // 2. 再做加减速限幅，保证每拍速度变化不会超过电机和机构更容易接受的范围
  auto smooth_axis = [&](double current, double desired) {
    const double blended = current + alpha * (desired - current);
    const double delta = blended - current;
    const double limit = std::abs(desired) < std::abs(current) ? decel_limit : accel_limit;
    return current + std::clamp(delta, -limit, limit);
  };

  smoothed_cmd.linear.x = smooth_axis(filtered_cmd_.linear.x, desired_cmd.linear.x);
  smoothed_cmd.linear.y = smooth_axis(filtered_cmd_.linear.y, desired_cmd.linear.y);
  smoothed_cmd.angular.z = 0.0;

  if (std::fabs(smoothed_cmd.linear.x) < 1e-3) {
    smoothed_cmd.linear.x = 0.0;
  }
  if (std::fabs(smoothed_cmd.linear.y) < 1e-3) {
    smoothed_cmd.linear.y = 0.0;
  }

  filtered_cmd_ = smoothed_cmd;
  return smoothed_cmd;
}

void BackUpFreeSpace::resetExecutionState()
{
  filtered_cmd_ = geometry_msgs::msg::Twist {};
  active_plan_ = EscapePlan();
  execution_state_ = RecoveryExecutionState::PLANNING;
  last_cycle_time_.reset();
  last_replan_time_.reset();
  blocked_cycles_ = 0;
  clear_cycles_ = 0;
  failed_replan_attempts_ = 0;
  command_distance_abs_ = 0.0;
  command_speed_abs_ = 0.0;
  active_plan_average_cost_ = 0.0;
  active_plan_source_ = PlanSource::CORRIDOR_PRIMARY;
  last_safe_prefix_distance_.reset();
  estimated_prefix_rate_ = 0.0;
}

bool BackUpFreeSpace::replanFromCurrentPose(
  const geometry_msgs::msg::PoseStamped & current_pose, double remaining_total_distance,
  double total_distance_traveled)
{
  nav2_msgs::msg::Costmap costmap;
  if (!fetchCostmap(costmap)) {
    return false;
  }

  // BLOCKED 后并不是立刻宣告失败，而是允许基于当前位置再次找一条更顺的恢复轨迹。
  // 这样可以更好应对：
  // 1. 动态障碍短时挡路
  // 2. 膨胀层边界变化
  // 3. 全向底盘当前已经略微侧移后的新局部几何关系
  EscapePlan new_plan;
  const auto pose_2d = poseToPose2D(current_pose);
  if (!planEscapeTrajectory(costmap, pose_2d, remaining_total_distance, new_plan)) {
    if (!planCentroidFallbackTrajectory(costmap, pose_2d, remaining_total_distance, new_plan)) {
      return false;
    }
    active_plan_source_ = PlanSource::CENTROID_FALLBACK;
  } else {
    active_plan_source_ = PlanSource::CORRIDOR_PRIMARY;
  }

  active_plan_ = new_plan;
  active_plan_average_cost_ = computePlanAverageCost(costmap, active_plan_);
  plan_start_pose_ = current_pose;
  completed_distance_before_plan_ = total_distance_traveled;
  previous_plan_heading_ = active_plan_.heading;
  has_previous_plan_heading_ = true;
  last_replan_time_ = clock_->now();
  RCLCPP_INFO(
    logger_,
    "Recovery replanned: source=%s remaining_total=%.2f released_segment=%.2f heading=%.2fdeg score=%.2f avg_cost=%.2f",
    planSourceName(active_plan_source_),
    remaining_total_distance, active_plan_.distance,
    active_plan_.heading * 180.0 / M_PI, active_plan_.score, active_plan_average_cost_);
  return true;
}

void BackUpFreeSpace::visualizePlan(
  const geometry_msgs::msg::Pose2D & pose, const EscapePlan & plan)
{
  if (!visualize_ || !marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = global_frame_;
  path_marker.header.stamp = clock_->now();
  path_marker.ns = "back_up_free_space";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.05;
  const bool centroid_fallback = active_plan_source_ == PlanSource::CENTROID_FALLBACK;
  path_marker.color.r = centroid_fallback ? 1.0f : 0.15f;
  path_marker.color.g = centroid_fallback ? 0.55f : 1.0f;
  path_marker.color.b = centroid_fallback ? 0.10f : 0.25f;
  path_marker.color.a = 0.95f;

  geometry_msgs::msg::Point start;
  start.x = pose.x;
  start.y = pose.y;
  start.z = 0.0;
  path_marker.points.push_back(start);
  for (const auto & point : plan.centerline) {
    path_marker.points.push_back(point);
  }
  markers.markers.push_back(path_marker);

  // 可视化恢复走廊左右边界：
  // 1. 方便在 RViz 里直接看出“恢复动作实际认为哪里是可通走廊”
  // 2. 便于验证分层采样 / 分段放行是否沿着预期走廊工作
  const double nx = -std::sin(plan.heading);
  const double ny = std::cos(plan.heading);

  visualization_msgs::msg::Marker left_boundary_marker;
  left_boundary_marker.header = path_marker.header;
  left_boundary_marker.ns = "back_up_free_space";
  left_boundary_marker.id = 2;
  left_boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  left_boundary_marker.action = visualization_msgs::msg::Marker::ADD;
  left_boundary_marker.scale.x = 0.03;
  left_boundary_marker.color.r = centroid_fallback ? 1.0f : 0.1f;
  left_boundary_marker.color.g = centroid_fallback ? 0.35f : 0.75f;
  left_boundary_marker.color.b = centroid_fallback ? 0.20f : 1.0f;
  left_boundary_marker.color.a = 0.9f;

  geometry_msgs::msg::Point start_left = start;
  start_left.x += nx * corridor_half_width_;
  start_left.y += ny * corridor_half_width_;
  left_boundary_marker.points.push_back(start_left);
  for (const auto & point : plan.centerline) {
    geometry_msgs::msg::Point left_point = point;
    left_point.x += nx * corridor_half_width_;
    left_point.y += ny * corridor_half_width_;
    left_boundary_marker.points.push_back(left_point);
  }
  markers.markers.push_back(left_boundary_marker);

  visualization_msgs::msg::Marker right_boundary_marker;
  right_boundary_marker.header = path_marker.header;
  right_boundary_marker.ns = "back_up_free_space";
  right_boundary_marker.id = 3;
  right_boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  right_boundary_marker.action = visualization_msgs::msg::Marker::ADD;
  right_boundary_marker.scale.x = 0.03;
  right_boundary_marker.color.r = centroid_fallback ? 1.0f : 0.1f;
  right_boundary_marker.color.g = centroid_fallback ? 0.35f : 0.75f;
  right_boundary_marker.color.b = centroid_fallback ? 0.20f : 1.0f;
  right_boundary_marker.color.a = 0.9f;

  geometry_msgs::msg::Point start_right = start;
  start_right.x -= nx * corridor_half_width_;
  start_right.y -= ny * corridor_half_width_;
  right_boundary_marker.points.push_back(start_right);
  for (const auto & point : plan.centerline) {
    geometry_msgs::msg::Point right_point = point;
    right_point.x -= nx * corridor_half_width_;
    right_point.y -= ny * corridor_half_width_;
    right_boundary_marker.points.push_back(right_point);
  }
  markers.markers.push_back(right_boundary_marker);

  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header = path_marker.header;
  goal_marker.ns = "back_up_free_space";
  goal_marker.id = 1;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position = plan.goal_point;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.18;
  goal_marker.scale.y = 0.18;
  goal_marker.scale.z = 0.18;
  goal_marker.color.r = centroid_fallback ? 1.0f : 0.95f;
  goal_marker.color.g = centroid_fallback ? 0.45f : 0.85f;
  goal_marker.color.b = centroid_fallback ? 0.15f : 0.15f;
  goal_marker.color.a = 0.95f;
  markers.markers.push_back(goal_marker);

  visualization_msgs::msg::Marker text_marker;
  text_marker.header = path_marker.header;
  text_marker.ns = "back_up_free_space";
  text_marker.id = 4;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.pose.position = plan.goal_point;
  text_marker.pose.position.z += 0.28;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.16;
  text_marker.color.r = centroid_fallback ? 1.0f : 0.85f;
  text_marker.color.g = centroid_fallback ? 0.50f : 0.95f;
  text_marker.color.b = centroid_fallback ? 0.15f : 0.90f;
  text_marker.color.a = 0.95f;
  text_marker.text =
    std::string(planSourceName(active_plan_source_)) +
    " cost=" + std::to_string(active_plan_average_cost_).substr(0, 4);
  markers.markers.push_back(text_marker);

  marker_pub_->publish(markers);
}

}  // namespace ats_nav2_behaviors

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ats_nav2_behaviors::BackUpFreeSpace, nav2_core::Behavior)
