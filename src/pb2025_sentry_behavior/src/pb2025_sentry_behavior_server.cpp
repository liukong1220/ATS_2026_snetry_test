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
: TreeExecutionServer(options)
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
  node()->get_parameter("decision.vision.topic", decision_vision_topic_);
  node()->get_parameter("decision.vision.timeout_s", decision_vision_timeout_s_);
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
  declare_parameter("decision.motion.default_spin_speed", 7.0);
  // 视觉侧接入参数单独放到 `decision.vision.*` 下，
  // 方便后续继续扩展超时、目标选择策略和启停开关。
  declare_parameter("decision.vision.topic", std::string("vision/target"));
  declare_parameter("decision.vision.timeout_s", 0.5);

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
  declare_parameter("decision.decision_config.decision_period_ms", 100);
  declare_parameter("decision.decision_config.goal_position_tolerance", 0.1);
  declare_parameter("decision.decision_config.waypoint_stop_duration_s", 0.0);
  declare_parameter("decision.decision_config.patrol_preview_points", 2);
  declare_parameter("decision.decision_config.action_server_wait_timeout_s", 0.5);
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
  double default_spin_speed = 7.0;
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
  node()->get_parameter("decision.motion.default_spin_speed", default_spin_speed);
  node()->get_parameter("decision.vision.timeout_s", decision_vision_timeout_s_);

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
  // 把视觉相关参数也放到黑板，
  // 这样 XML 就可以直接通过 `{@decision_gimbal_topic}` / `{@decision_vision_timeout_s}` 引用。
  globalBlackboard()->set("decision_gimbal_topic", gimbal_topic);
  globalBlackboard()->set("decision_default_spin_speed", default_spin_speed);
  globalBlackboard()->set("decision_vision_timeout_s", decision_vision_timeout_s_);
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
  return std::nullopt;
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
