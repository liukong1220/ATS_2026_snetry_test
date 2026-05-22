#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>

#include "behaviortree_cpp/xml_parsing.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pb_rm_interfaces/msg/buff.hpp"
#include "pb_rm_interfaces/msg/event_data.hpp"
#include "pb_rm_interfaces/msg/game_robot_hp.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/ground_robot_position.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "sp_msgs/msg/vision_target_msg.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

namespace
{

std::string normalizeToken(std::string value)
{
  value.erase(
    std::remove_if(
      value.begin(), value.end(),
      [](unsigned char ch) { return std::isspace(ch) != 0; }),
    value.end());
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return value;
}

std::string sanitizeInputSource(const std::string & raw_source, const rclcpp::Logger & logger)
{
  const auto source = normalizeToken(raw_source);
  if (source == "referee" || source == "simulation") {
    return source;
  }

  RCLCPP_WARN(
    logger,
    "Unsupported decision.input_source='%s', fallback to 'referee'",
    raw_source.c_str());
  return "referee";
}

bool isSupportedSimulationMode(const std::string & mode)
{
  return mode == "patrol" || mode == "anchor" || mode == "retreat" || mode == "safe";
}

const char * gameProgressName(const uint8_t progress)
{
  switch (progress) {
    case pb_rm_interfaces::msg::GameStatus::NOT_START:
      return "NOT_START";
    case pb_rm_interfaces::msg::GameStatus::PREPARATION:
      return "PREPARATION";
    case pb_rm_interfaces::msg::GameStatus::SELF_CHECKING:
      return "SELF_CHECKING";
    case pb_rm_interfaces::msg::GameStatus::COUNT_DOWN:
      return "COUNT_DOWN";
    case pb_rm_interfaces::msg::GameStatus::RUNNING:
      return "RUNNING";
    case pb_rm_interfaces::msg::GameStatus::GAME_OVER:
      return "GAME_OVER";
    default:
      return "UNKNOWN";
  }
}

std::string sanitizeSimulationMode(
  const std::string & raw_mode, const std::string & fallback, const rclcpp::Logger & logger,
  const std::string & context)
{
  const auto mode = normalizeToken(raw_mode);
  if (isSupportedSimulationMode(mode)) {
    return mode;
  }

  RCLCPP_WARN(
    logger,
    "Unsupported %s='%s', fallback to '%s'",
    context.c_str(), raw_mode.c_str(), fallback.c_str());
  return fallback;
}

}  // namespace

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(node()->get_clock())),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node(), false))
{
  node()->declare_parameter("use_cout_logger", false);
  node()->declare_parameter("export_tree_models_on_shutdown", false);
  node()->declare_parameter(
    "tree_models_output_path",
    (std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml").string());
  declareDecisionParameters();
  node()->get_parameter("use_cout_logger", use_cout_logger_);
  node()->get_parameter("export_tree_models_on_shutdown", export_tree_models_on_shutdown_);
  node()->get_parameter("tree_models_output_path", tree_models_output_path_);
  node()->get_parameter("decision.input_source", decision_input_source_);
  node()->get_parameter("decision.simulation.default_mode", decision_sim_mode_);
  node()->get_parameter("decision.simulation.mode_topic", decision_sim_mode_topic_);
  node()->get_parameter("decision.topics.gimbal_cmd", decision_gimbal_topic_);
  node()->get_parameter("decision.topics.robot_mode", decision_robot_mode_topic_);
  node()->get_parameter("decision.vision.topic", decision_vision_topic_);
  node()->get_parameter("decision.vision.timeout_s", decision_vision_timeout_s_);
  node()->get_parameter("decision.motion.hit_spin_speed", decision_hit_spin_speed_);
  node()->get_parameter("decision.pose.expected_frame", pose_expected_frame_);
  node()->get_parameter("decision.pose.timeout_s", pose_timeout_s_);
  node()->get_parameter("decision.pose.tf_fallback_enabled", pose_tf_fallback_enabled_);
  decision_input_source_ = sanitizeInputSource(decision_input_source_, node()->get_logger());
  decision_sim_mode_ = sanitizeSimulationMode(
    decision_sim_mode_, "patrol", node()->get_logger(),
    "parameter decision.simulation.default_mode");
  globalBlackboard()->set("node", node());
  globalBlackboard()->set("decision_input_source", decision_input_source_);
  globalBlackboard()->set("decision_sim_mode", decision_sim_mode_);

  subscribe<pb_rm_interfaces::msg::EventData>("referee/event_data", "referee_eventData");
  subscribe<pb_rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp", "referee_allRobotHP");
  subscribe<pb_rm_interfaces::msg::GameStatus>("referee/game_status", "referee_gameStatus");
  subscribe<pb_rm_interfaces::msg::GroundRobotPosition>(
    "referee/ground_robot_position", "referee_groundRobotPosition");
  subscribe<pb_rm_interfaces::msg::RfidStatus>("referee/rfid_status", "referee_rfidStatus");
  subscribe<pb_rm_interfaces::msg::RobotStatus>("referee/robot_status", "referee_robotStatus");
  subscribe<pb_rm_interfaces::msg::Buff>("referee/buff", "referee_buff");
  // 视觉融合状态直接进入根黑板，供行为树条件节点统一消费。
  subscribe<sp_msgs::msg::VisionTargetMsg>(decision_vision_topic_, "sp_vision_target");

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);

  auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg->header;
      pose.pose = msg->pose.pose;
      // 优先沿用 odom 订阅位姿，保证正常行驶时规划用的是导航反馈主链路。
      globalBlackboard()->set("decision_current_pose", pose);
    };
  subscriptions_.push_back(
    node()->create_subscription<nav_msgs::msg::Odometry>("odom", 10, odom_callback));
  subscriptions_.push_back(
    node()->create_subscription<nav_msgs::msg::Odometry>("odometry", 10, odom_callback));

  auto sim_mode_callback = [this](const std_msgs::msg::String::SharedPtr msg) {
      const auto normalized_mode = normalizeToken(msg->data);
      if (!isSupportedSimulationMode(normalized_mode)) {
        RCLCPP_WARN(
          node()->get_logger(),
          "Ignore unsupported decision simulation mode '%s' on topic '%s'",
          msg->data.c_str(), decision_sim_mode_topic_.c_str());
        return;
      }

      if (decision_sim_mode_ != normalized_mode) {
        RCLCPP_INFO(
          node()->get_logger(), "Decision simulation mode switched: %s -> %s",
          decision_sim_mode_.c_str(), normalized_mode.c_str());
      }
      decision_sim_mode_ = normalized_mode;
      globalBlackboard()->set("decision_sim_mode", decision_sim_mode_);
    };
  subscriptions_.push_back(
    node()->create_subscription<std_msgs::msg::String>(
      decision_sim_mode_topic_, 10, sim_mode_callback));
}

void SentryBehaviorServer::declareDecisionParameters()
{
  auto declare_parameter = [this](const std::string & name, auto default_value) {
      node()->declare_parameter(name, default_value);
    };

  declare_parameter("decision.goal_points.x", std::vector<double>{0.2, 5.38, 3.6, 0.0});
  declare_parameter("decision.goal_points.y", std::vector<double>{0.18, 2.2, 3.6, 0.0});
  declare_parameter("decision.goal_points.z", std::vector<double>{0.0, 0.0, 0.0, 0.0});

  declare_parameter("decision.point_roles.supply_safe_point_index", 0);
  declare_parameter("decision.point_roles.patrol_indices", std::vector<int64_t>{1, 2});
  declare_parameter(
    "decision.point_roles.low_hp_candidate_indices", std::vector<int64_t>{1, 2});
  declare_parameter("decision.point_roles.critical_time_target_index", 1);
  declare_parameter("decision.point_roles.anchor_target_index", 2);

  declare_parameter("decision.input_source", std::string("referee"));
  declare_parameter("decision.simulation.default_mode", std::string("patrol"));
  declare_parameter("decision.simulation.mode_topic", std::string("decision/sim_mode"));

  declare_parameter("decision.referee.start_gate.expected_game_progress", 4);
  declare_parameter("decision.referee.start_gate.min_remain_time", 0);
  declare_parameter("decision.referee.start_gate.max_remain_time", 420);

  declare_parameter("decision.topics.spin", std::string("cmd_spin"));
  declare_parameter("decision.topics.cmd_vel", std::string("cmd_vel"));
  declare_parameter("decision.topics.gimbal_cmd", std::string("cmd_gimbal"));
  declare_parameter("decision.topics.robot_mode", std::string("decision/robot_mode"));
  declare_parameter("decision.motion.default_spin_speed", 0.0);
  declare_parameter("decision.motion.hit_spin_speed", 7.0);
  declare_parameter("decision.motion.hit_spin_stop_after_no_hp_drop_s", 2.0);
  declare_parameter("decision.motion.hit_minimum_spin_duration_s", 1.2);
  declare_parameter("decision.mode_limits.switch_cooldown_s", 5.0);
  declare_parameter("decision.mode_limits.max_cumulative_s", 180.0);
  declare_parameter("decision.mode_visualization.enabled", true);
  declare_parameter(
    "decision.mode_visualization.topic", std::string("decision/robot_mode_markers"));
  // 视觉侧接入参数单独放到 `decision.vision.*` 下，
  // 方便后续继续扩展超时、目标选择策略和启停开关。
  declare_parameter("decision.vision.topic", std::string("vision/target"));
  declare_parameter("decision.vision.timeout_s", 0.5);
  declare_parameter("decision.vision.attack_radius", 2.0);
  declare_parameter("decision.vision.follow_occupied_threshold", 50);
  declare_parameter("decision.vision.follow_sample_count", 16);
  declare_parameter("decision.vision.follow_arc_half_angle_deg", 90.0);
  declare_parameter("decision.vision.min_replan_interval_s", 0.4);
  declare_parameter("decision.vision.min_goal_shift_m", 0.35);
  declare_parameter("decision.vision.min_goal_distance_from_robot_m", 0.35);
  // 跟随点选侧稳定参数：
  // 1. prefer_previous_goal_side：若上一帧已经有稳定可用的跟随点，优先保持在同一侧，
  //    减少在转角、终点附近或目标轻微抖动时左右突然翻边。
  // 2. max_target_shift_for_side_hold_m：仅当敌方地图点变化较小时才保持旧侧，
  //    若敌方已经明显移动，则允许重新选更合适的一侧。
  // 3. max_goal_angle_step_deg：即使本帧重新选出的圆周目标发生了较大角度跃迁，
  //    也只允许每个决策周期沿圆周前进有限角度，避免目标点瞬间跳到另一侧。
  // 4. pose_jump_reset_distance_m：若当前车位相对上一拍参考位姿出现明显突变，
  //    直接清空旧的视觉平滑缓存，保证重定位后立即重新按“当前车位最近圆周点”选目标。
  declare_parameter("decision.vision.prefer_previous_goal_side", true);
  declare_parameter("decision.vision.max_target_shift_for_side_hold_m", 0.8);
  declare_parameter("decision.vision.max_goal_angle_step_deg", 18.0);
  declare_parameter("decision.vision.pose_jump_reset_distance_m", 0.8);
  // 视觉跟随平滑接管参数：
  // 1. activation_hold_s：首次看到目标后，先稳定保持一小段时间再接管。
  // 2. switch_target_hold_s：切换敌方目标前，要求新目标持续稳定一段时间。
  // 3. override_hold_s：短时丢帧/遮挡时，继续保持当前视觉接管，避免立刻掉回巡逻。
  declare_parameter("decision.vision.activation_hold_s", 0.25);
  declare_parameter("decision.vision.switch_target_hold_s", 0.45);
  declare_parameter("decision.vision.override_hold_s", 0.6);
  declare_parameter("decision.vision.visualization_enabled", true);
  declare_parameter(
    "decision.vision.visualization_topic", std::string("decision/vision_follow_markers"));
  // 资源策略统一决定当前是否允许视觉接管：
  // 1. defend   : 血量已经低到必须保命，优先退防。
  // 2. resupply : 血量/弹量不健康，退出追击并回补给安全点。
  // 3. engage   : 血量/弹量都健康，允许巡逻与视觉接管。
  //
  // enter/exit 成对出现是为了形成迟滞，避免数值卡在阈值附近来回横跳。
  declare_parameter("decision.resource_policy.defend_enter_hp", 100);
  declare_parameter("decision.resource_policy.defend_exit_hp", 150);
  declare_parameter("decision.resource_policy.resupply_enter_hp", 250);
  declare_parameter("decision.resource_policy.resupply_exit_hp", 300);
  declare_parameter("decision.resource_policy.resupply_enter_ammo", 50);
  declare_parameter("decision.resource_policy.resupply_exit_ammo", 100);
  declare_parameter("decision.resource_policy.assume_engage_when_status_missing", false);
  declare_parameter("decision.pose.expected_frame", std::string("map"));
  declare_parameter("decision.pose.timeout_s", 0.5);
  declare_parameter("decision.pose.tf_fallback_enabled", true);

  declare_parameter("decision.time_thresholds.abundant", 300);
  declare_parameter("decision.time_thresholds.normal", 240);
  declare_parameter("decision.time_thresholds.tense", 120);
  declare_parameter("decision.time_thresholds.critical", 40);

  declare_parameter("decision.hp_thresholds.abundant.high", 400);
  declare_parameter("decision.hp_thresholds.abundant.medium", 220);
  declare_parameter("decision.hp_thresholds.abundant.low", 120);

  declare_parameter("decision.hp_thresholds.normal.high", 400);
  declare_parameter("decision.hp_thresholds.normal.medium", 250);
  declare_parameter("decision.hp_thresholds.normal.low", 150);

  declare_parameter("decision.hp_thresholds.tense.high", 400);
  declare_parameter("decision.hp_thresholds.tense.medium", 250);
  declare_parameter("decision.hp_thresholds.tense.low", 150);

  declare_parameter("decision.hp_thresholds.critical.high", 400);
  declare_parameter("decision.hp_thresholds.critical.medium", 250);
  declare_parameter("decision.hp_thresholds.critical.low", 150);

  declare_parameter("decision.decision_config.path_tolerance", 0.2);
  declare_parameter(
    "decision.decision_config.nav2_action_server", std::string("/navigate_through_poses"));
  declare_parameter(
    "decision.decision_config.nav2_to_pose_action_server", std::string("/navigate_to_pose"));
  declare_parameter("decision.decision_config.decision_period_ms", 100);
  declare_parameter("decision.decision_config.goal_position_tolerance", 0.1);
  // 行为层“是否已到达路径终点”的判定容差。
  // 这个值应尽量与 Nav2 的 general_goal_checker.xy_goal_tolerance 对齐，
  // 否则会出现 Nav2 已经报 Goal succeeded，但行为层仍继续重发同一路径。
  declare_parameter("decision.decision_config.path_goal_reached_tolerance", 0.1);
  // 导航目标重发节流参数：
  // 1. active_goal_hold_tolerance：若新路径与当前已发给 Nav2 的路径仅有小范围偏差，
  //    则先认为它们属于“同一个意图”，不要立刻 cancel + 重发。
  // 2. active_goal_min_resend_interval_s：即使目标有轻微变化，也要求与上一次发目标
  //    至少间隔这么久才允许再次 preempt，减少视觉抖动放大成 MPPI 左右试探。
  declare_parameter("decision.decision_config.active_goal_hold_tolerance", 0.45);
  declare_parameter("decision.decision_config.active_goal_min_resend_interval_s", 0.8);
  // 视觉实时追随专用节流参数：
  // 视觉分支会持续围绕敌方目标圆周刷新“距离当前车位最近的点”，
  // 因此不能完全复用普通巡逻的宽容差，否则新圆周点会被误判成旧目标附近的小抖动。
  declare_parameter("decision.decision_config.vision_active_goal_hold_tolerance", 0.18);
  declare_parameter("decision.decision_config.vision_active_goal_min_resend_interval_s", 0.25);
  declare_parameter("decision.decision_config.waypoint_stop_duration_s", 0.0);
  declare_parameter("decision.decision_config.patrol_preview_points", 1);
  declare_parameter("decision.decision_config.action_server_wait_timeout_s", 0.5);

  declare_parameter("decision.rmuc.csv_waypoints_file", std::string("params/rmuc_waypoints.csv"));
  declare_parameter(
    "decision.rmuc.patrol_csv_file", std::string("params/rmuc_patrol_waypoints.csv"));
  declare_parameter("decision.rmuc.endgame_time_threshold", 180);
  declare_parameter("decision.rmuc.supply_point.x", -4.30);
  declare_parameter("decision.rmuc.supply_point.y", -2.00);
  declare_parameter("decision.rmuc.supply_point.z", 0.0);
}

void SentryBehaviorServer::initializeDecisionBlackboard()
{
  int supply_safe_point_index = 0;
  int critical_time_target_index = 1;
  int anchor_target_index = 2;
  int game_start_expected_progress = 4;
  int game_start_min_remain_time = 0;
  int game_start_max_remain_time = 420;
  std::string spin_topic = "cmd_spin";
  std::string cmd_vel_topic = "cmd_vel";
  std::string gimbal_topic = "cmd_gimbal";
  std::string robot_mode_topic = "decision/robot_mode";
  double default_spin_speed = 0.0;
  double hit_spin_speed = 7.0;
  double hit_spin_stop_after_no_hp_drop_s = 2.0;
  double hit_minimum_spin_duration_s = 1.2;
  double mode_switch_cooldown_s = 5.0;
  double mode_max_cumulative_s = 180.0;
  int resupply_enter_hp = 250;
  int rmuc_endgame_time_threshold = 180;
  std::string rmuc_csv_file = "params/rmuc_waypoints.csv";
  std::string rmuc_patrol_csv_file = "params/rmuc_patrol_waypoints.csv";
  node()->get_parameter(
    "decision.point_roles.supply_safe_point_index", supply_safe_point_index);
  node()->get_parameter(
    "decision.point_roles.critical_time_target_index", critical_time_target_index);
  node()->get_parameter(
    "decision.point_roles.anchor_target_index", anchor_target_index);
  node()->get_parameter(
    "decision.referee.start_gate.expected_game_progress", game_start_expected_progress);
  node()->get_parameter(
    "decision.referee.start_gate.min_remain_time", game_start_min_remain_time);
  node()->get_parameter(
    "decision.referee.start_gate.max_remain_time", game_start_max_remain_time);
  node()->get_parameter("decision.topics.spin", spin_topic);
  node()->get_parameter("decision.topics.cmd_vel", cmd_vel_topic);
  node()->get_parameter("decision.topics.gimbal_cmd", gimbal_topic);
  // 姿态模式话题：行为树发布，standard_robot_pp_ros2 订阅后再下发给下位机。
  node()->get_parameter("decision.topics.robot_mode", robot_mode_topic);
  node()->get_parameter("decision.motion.default_spin_speed", default_spin_speed);
  // 受击时的自旋速度。
  node()->get_parameter("decision.motion.hit_spin_speed", hit_spin_speed);
  // 最近一次掉血后，若在该时长内没有新的掉血，则停止自旋。
  node()->get_parameter(
    "decision.motion.hit_spin_stop_after_no_hp_drop_s", hit_spin_stop_after_no_hp_drop_s);
  node()->get_parameter(
    "decision.motion.hit_minimum_spin_duration_s", hit_minimum_spin_duration_s);
  // 姿态切换冷却和单局累计时长上限，由 PublishRobotMode 统一执行。
  node()->get_parameter("decision.mode_limits.switch_cooldown_s", mode_switch_cooldown_s);
  node()->get_parameter("decision.mode_limits.max_cumulative_s", mode_max_cumulative_s);
  node()->get_parameter("decision.vision.timeout_s", decision_vision_timeout_s_);
  node()->get_parameter("decision.resource_policy.resupply_enter_hp", resupply_enter_hp);
  node()->get_parameter("decision.rmuc.csv_waypoints_file", rmuc_csv_file);
  node()->get_parameter("decision.rmuc.patrol_csv_file", rmuc_patrol_csv_file);
  node()->get_parameter("decision.rmuc.endgame_time_threshold", rmuc_endgame_time_threshold);

  globalBlackboard()->set("node", node());
  globalBlackboard()->set("decision_input_source", decision_input_source_);
  globalBlackboard()->set("decision_sim_mode", decision_sim_mode_);
  globalBlackboard()->set("decision_supply_safe_point_index", supply_safe_point_index);
  globalBlackboard()->set("decision_critical_time_target_index", critical_time_target_index);
  globalBlackboard()->set("decision_anchor_target_index", anchor_target_index);
  globalBlackboard()->set(
    "decision_game_start_expected_progress", game_start_expected_progress);
  globalBlackboard()->set(
    "decision_game_start_min_remain_time", game_start_min_remain_time);
  globalBlackboard()->set(
    "decision_game_start_max_remain_time", game_start_max_remain_time);
  globalBlackboard()->set("decision_spin_topic", spin_topic);
  globalBlackboard()->set("decision_cmd_vel_topic", cmd_vel_topic);
  // 把行为树运行时会直接引用的参数注入黑板，
  // 这样 XML 可以通过 {@...} 形式直接取值，避免把常量写死在树文件中。
  globalBlackboard()->set("decision_gimbal_topic", gimbal_topic);
  // 姿态模式相关黑板参数，供 PublishRobotMode / XML 直接引用。
  globalBlackboard()->set("decision_robot_mode_topic", robot_mode_topic);
  globalBlackboard()->set("decision_default_spin_speed", default_spin_speed);
  globalBlackboard()->set("decision_hit_spin_speed", hit_spin_speed);
  globalBlackboard()->set(
    "decision_hit_spin_stop_after_no_hp_drop_s", hit_spin_stop_after_no_hp_drop_s);
  globalBlackboard()->set(
    "decision_hit_minimum_spin_duration_s", hit_minimum_spin_duration_s);
  globalBlackboard()->set("decision_mode_switch_cooldown_s", mode_switch_cooldown_s);
  globalBlackboard()->set("decision_mode_max_cumulative_s", mode_max_cumulative_s);
  globalBlackboard()->set("decision_vision_timeout_s", decision_vision_timeout_s_);
  globalBlackboard()->set("decision_resupply_enter_hp", resupply_enter_hp);
  globalBlackboard()->set("decision_rmuc_csv_file", rmuc_csv_file);
  globalBlackboard()->set("decision_rmuc_patrol_csv_file", rmuc_patrol_csv_file);
  globalBlackboard()->set("decision_rmuc_endgame_time_threshold", rmuc_endgame_time_threshold);
  // 资源策略节点会在运行时持续覆写该值，这里先给一个明确的初值，方便调试观测。
  globalBlackboard()->set("decision_resource_mode", std::string("unknown"));
  globalBlackboard()->set("decision_requested_robot_mode", std::string("unknown"));
  globalBlackboard()->set("decision_active_robot_mode", std::string("unknown"));
  globalBlackboard()->set("decision_patrol_cursor", 0);
  globalBlackboard()->set("decision_patrol_direction", 1);
  globalBlackboard()->set("decision_next_patrol_cursor", 0);
  globalBlackboard()->set("decision_next_patrol_direction", 1);
  globalBlackboard()->set("decision_low_hp_target_index", -1);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(),
    "onGoalReceived with tree name '%s' with payload '%s' (input_source=%s, sim_mode=%s)",
    tree_name.c_str(), payload.c_str(), decision_input_source_.c_str(), decision_sim_mode_.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
  initializeDecisionBlackboard();
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  logDecisionSnapshot();

  if (!pose_tf_fallback_enabled_) {
    return std::nullopt;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  bool has_pose = globalBlackboard()->get("decision_current_pose", current_pose);
  bool pose_stale = !has_pose;
  if (has_pose && pose_timeout_s_ > 0.0) {
    const rclcpp::Time pose_stamp(current_pose.header.stamp);
    if (pose_stamp.nanoseconds() > 0) {
      pose_stale = (node()->now() - pose_stamp).seconds() > pose_timeout_s_;
    }
  }

  if (!pose_stale) {
    return std::nullopt;
  }

  for (const auto & base_frame : pose_base_frame_candidates_) {
    try {
      // loopback 手动改 initialpose、机器人短暂停车或 odom 发布停滞时，
      // TF 往往比 odom 更及时；这里用 TF 刷新黑板位姿，避免视觉跟随和到点判断卡死。
      const auto transform =
        tf_buffer_->lookupTransform(pose_expected_frame_, base_frame, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped fallback_pose;
      fallback_pose.header = transform.header;
      fallback_pose.pose.position.x = transform.transform.translation.x;
      fallback_pose.pose.position.y = transform.transform.translation.y;
      fallback_pose.pose.position.z = transform.transform.translation.z;
      fallback_pose.pose.orientation = transform.transform.rotation;
      globalBlackboard()->set("decision_current_pose", fallback_pose);
      RCLCPP_DEBUG_THROTTLE(
        node()->get_logger(), *node()->get_clock(), 2000,
        "Refresh decision_current_pose from TF: %s -> %s",
        pose_expected_frame_.c_str(), base_frame.c_str());
      return std::nullopt;
    } catch (const tf2::TransformException &) {
    }
  }

  RCLCPP_WARN_THROTTLE(
    node()->get_logger(), *node()->get_clock(), 2000,
    "decision_current_pose is stale and TF fallback failed for frame '%s'",
    pose_expected_frame_.c_str());
  return std::nullopt;
}

void SentryBehaviorServer::logDecisionSnapshot()
{
  pb_rm_interfaces::msg::GameStatus game_status;
  pb_rm_interfaces::msg::RobotStatus robot_status;
  if (
    !globalBlackboard()->get("referee_gameStatus", game_status) ||
    !globalBlackboard()->get("referee_robotStatus", robot_status))
  {
    RCLCPP_INFO_THROTTLE(
      node()->get_logger(), *node()->get_clock(), 3000,
      "[decision/summary] waiting referee data");
    return;
  }

  std::string resource_mode = "unknown";
  std::string requested_robot_mode = "unknown";
  std::string active_robot_mode = "unknown";
  auto bb = globalBlackboard();
  (void)bb->get("decision_resource_mode", resource_mode);
  (void)bb->get("decision_requested_robot_mode", requested_robot_mode);
  (void)bb->get("decision_active_robot_mode", active_robot_mode);

  std::string summary =
    std::string("[decision/summary] progress=") + gameProgressName(game_status.game_progress) +
    "(" + std::to_string(game_status.game_progress) + ")" +
    " remain=" + std::to_string(game_status.stage_remain_time) +
    " hp=" + std::to_string(robot_status.current_hp) + "/" + std::to_string(robot_status.maximum_hp) +
    " ammo=" + std::to_string(robot_status.projectile_allowance_17mm) +
    " heat=" + std::to_string(robot_status.shooter_17mm_1_barrel_heat) +
    " resource=" + resource_mode +
    " requested_mode=" + requested_robot_mode +
    " active_mode=" + active_robot_mode +
    " input=" + decision_input_source_ +
    " sim=" + decision_sim_mode_;

  if (summary != last_decision_summary_) {
    RCLCPP_INFO(node()->get_logger(), "%s", summary.c_str());
    last_decision_summary_ = summary;
  } else {
    RCLCPP_INFO_THROTTLE(
      node()->get_logger(), *node()->get_clock(), 3000,
      "%s", summary.c_str());
  }
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  if (action_server->shouldExportTreeModels()) {
    std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());
    std::ofstream file(action_server->treeModelsOutputPath());
    if (file.is_open()) {
      file << xml_models;
      RCLCPP_INFO(
        action_server->node()->get_logger(), "BehaviorTree models exported to %s",
        action_server->treeModelsOutputPath().c_str());
    } else {
      RCLCPP_WARN(
        action_server->node()->get_logger(), "Failed to export BehaviorTree models to %s",
        action_server->treeModelsOutputPath().c_str());
    }
  }

  rclcpp::shutdown();
}
