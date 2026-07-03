#include "ats_sentry_behavior/plugins/action/pub_robot_mode.hpp"

#include <algorithm>
#include <array>
#include <cctype>

#include "ats_sentry_behavior/decision_utils.hpp"
#include "ats_rm_interfaces/msg/game_status.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ats_sentry_behavior
{

namespace
{

// 下位机姿态枚举约定，必须与 standard_robot_pp_ros2 中的协议定义保持一致。
constexpr uint8_t kMoveMode = 3;
constexpr uint8_t kAttackMode = 1;
constexpr uint8_t kDefendMode = 2;
constexpr double kDefaultModeSwitchCooldownSeconds = 5.0;
constexpr double kDefaultModeMaxCumulativeSeconds = 180.0;
constexpr char kRobotModeRuntimeStateKey[] = "decision_robot_mode_runtime_state";
constexpr char kDefaultRobotModeVisualizationTopic[] = "decision/robot_mode_markers";

struct RobotModeRuntimeState
{
  // 是否已经初始化过本局姿态运行时状态。
  bool initialized = false;
  // 当前是否处于比赛 RUNNING 阶段，仅 RUNNING 阶段累计姿态时长。
  bool match_running = false;
  uint8_t last_game_progress = ats_rm_interfaces::msg::GameStatus::NOT_START;
  // 当前真正生效的姿态，不一定等于本 tick 请求的姿态。
  uint8_t active_mode = kMoveMode;
  // 上次累计时长更新的时间戳。
  int64_t last_update_ns = 0;
  // 上次成功切换姿态的时间戳，用于执行切换冷却。
  int64_t last_switch_ns = 0;
  // 单局累计时长，索引 0/1/2 分别对应 move/attack/defend。
  std::array<double, 3> cumulative_s{0.0, 0.0, 0.0};
};

uint8_t parseRobotMode(const std::string & raw_mode)
{
  // 行为树 XML 中使用字符串描述姿态，这里统一映射成协议约定的数值枚举。
  if (raw_mode == "attack") {
    return kAttackMode;
  }
  if (raw_mode == "defend" || raw_mode == "defense") {
    return kDefendMode;
  }
  return kMoveMode;
}

const char * modeName(uint8_t mode)
{
  switch (mode) {
    case kAttackMode:
      return "attack";
    case kDefendMode:
      return "defend";
    case kMoveMode:
    default:
      return "move";
  }
}

std::size_t modeIndex(uint8_t mode)
{
  // cumulative_s 只开了 3 个槽位，非法值统一夹紧到最后一个可用索引。
  return std::min<std::size_t>(mode, 2U);
}

// 判断某一姿态在当前这局比赛里是否仍可用。
bool isModeAvailable(
  const RobotModeRuntimeState & state, uint8_t mode, double max_cumulative_s)
{
  return state.cumulative_s[modeIndex(mode)] < max_cumulative_s;
}

// 初始化或重置一局比赛的姿态运行时状态。
void initializeRuntimeState(
  RobotModeRuntimeState & state, int64_t now_ns, bool match_running, uint8_t game_progress,
  double cooldown_s)
{
  // 每次新开一局比赛时，姿态累计时长都要从零开始重新统计。
  state.initialized = true;
  state.match_running = match_running;
  state.last_game_progress = game_progress;
  state.active_mode = kMoveMode;
  state.last_update_ns = now_ns;
  state.last_switch_ns = now_ns - static_cast<int64_t>(cooldown_s * 1e9);
  state.cumulative_s = {0.0, 0.0, 0.0};
}

// 只在比赛 RUNNING 期间累计当前激活姿态的使用时长。
void accumulateModeUsage(RobotModeRuntimeState & state, int64_t now_ns)
{
  const int64_t elapsed_ns = std::max<int64_t>(0, now_ns - state.last_update_ns);
  if (state.match_running && elapsed_ns > 0) {
    state.cumulative_s[modeIndex(state.active_mode)] += static_cast<double>(elapsed_ns) / 1e9;
  }
  state.last_update_ns = now_ns;
}

// 当目标姿态不可用时，按既定优先级选出仍然合法的回退姿态。
uint8_t selectAvailableMode(
  const RobotModeRuntimeState & state, uint8_t preferred_mode, double max_cumulative_s)
{
  std::array<bool, 3> visited{false, false, false};
  const std::array<uint8_t, 5> candidates = {
    preferred_mode, state.active_mode, kMoveMode, kAttackMode, kDefendMode};

  for (const auto candidate : candidates) {
    const auto index = modeIndex(candidate);
    if (visited[index]) {
      continue;
    }
    visited[index] = true;
    if (isModeAvailable(state, candidate, max_cumulative_s)) {
      return candidate;
    }
  }

  return kMoveMode;
}

}  // namespace

PublishRobotModeAction::PublishRobotModeAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  std::string visualization_topic = kDefaultRobotModeVisualizationTopic;
  node_->get_parameter("decision.mode_visualization.enabled", visualization_enabled_);
  node_->get_parameter("decision.mode_visualization.topic", visualization_topic);
  if (visualization_enabled_) {
    visualization_publisher_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic, 10);
  }
}

BT::PortsList PublishRobotModeAction::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<std::string>(
      "mode", "move",
      "当前分支期望的姿态字符串，支持 move / attack / defend"),
    BT::InputPort<ats_rm_interfaces::msg::GameStatus>(
      "game_status", "{@referee_gameStatus}",
      "比赛状态。用于识别新的一局开始，并清空单局姿态累计时长"),
    BT::InputPort<double>(
      "cooldown_s", "{@decision_mode_switch_cooldown_s}",
      "姿态切换冷却时间。距离上一次成功切换不足该时间时，不允许切到新姿态"),
    BT::InputPort<double>(
      "max_cumulative_s", "{@decision_mode_max_cumulative_s}",
      "单局比赛内某一种姿态允许累计使用的最大时长"),
  });
}

bool PublishRobotModeAction::setMessage(example_interfaces::msg::UInt8 & msg)
{
  std::string mode = "move";
  getInput("mode", mode);
  // 统一转成小写，避免 XML 中大小写写法不一致导致解析错误。
  std::transform(
    mode.begin(), mode.end(), mode.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  const uint8_t requested_mode = parseRobotMode(mode);
  // 这里发出去的不一定是 requested_mode，
  // 还要经过冷却时间和累计时长限制后的最终裁决。
  if (auto root_blackboard = config().blackboard->rootBlackboard()) {
    root_blackboard->set("decision_requested_robot_mode", std::string(modeName(requested_mode)));
  }
  msg.data = resolveModeWithConstraints(requested_mode);
  return true;
}

bool PublishRobotModeAction::setHaltMessage(example_interfaces::msg::UInt8 & msg)
{
  msg.data = kMoveMode;
  return true;
}

uint8_t PublishRobotModeAction::resolveModeWithConstraints(uint8_t requested_mode)
{
  auto root_blackboard = config().blackboard->rootBlackboard();
  if (root_blackboard == nullptr) {
    // 没有根黑板时无法读取/保存运行时状态，只能直接返回请求值。
    return requested_mode;
  }

  const auto node = decision::getNodeFromBlackboard(*this);
  const int64_t now_ns = node->now().nanoseconds();

  double cooldown_s = kDefaultModeSwitchCooldownSeconds;
  getInput("cooldown_s", cooldown_s);

  double max_cumulative_s = kDefaultModeMaxCumulativeSeconds;
  getInput("max_cumulative_s", max_cumulative_s);

  RobotModeRuntimeState state;
  auto game_status = getInput<ats_rm_interfaces::msg::GameStatus>("game_status");
  const bool has_game_status = static_cast<bool>(game_status);
  const uint8_t current_game_progress =
    has_game_status ? game_status->game_progress : ats_rm_interfaces::msg::GameStatus::NOT_START;
  const bool match_running =
    has_game_status && current_game_progress == ats_rm_interfaces::msg::GameStatus::RUNNING;

  // 注意：
  // 1. 只有比赛 RUNNING 期间才累计姿态时长；
  // 2. 比赛从非 RUNNING 重新进入 RUNNING，视为新的一局，累计时间清零；
  // 3. active_mode 表示当前真正生效并已经下发给下位机的姿态。

  // 第一次运行，或者检测到比赛从非 RUNNING 重新进入 RUNNING，都视为新的一局。
  if (!root_blackboard->get(kRobotModeRuntimeStateKey, state) || !state.initialized) {
    initializeRuntimeState(state, now_ns, match_running, current_game_progress, cooldown_s);
  } else if (!state.match_running && match_running) {
    initializeRuntimeState(state, now_ns, true, current_game_progress, cooldown_s);
  } else {
    accumulateModeUsage(state, now_ns);
    state.match_running = match_running;
    state.last_game_progress = current_game_progress;
  }

  uint8_t resolved_mode = requested_mode;
  const bool active_mode_exhausted = !isModeAvailable(state, state.active_mode, max_cumulative_s);
  if (active_mode_exhausted) {
    // 当前姿态已经被累计时长规则“封禁”，不能继续保留，只能重新挑选合法姿态。
    resolved_mode = selectAvailableMode(state, requested_mode, max_cumulative_s);
  } else {
    if (!isModeAvailable(state, requested_mode, max_cumulative_s)) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 2000,
        "Robot posture '%s' reached cumulative limit %.1fs, keep '%s' instead",
        modeName(requested_mode), max_cumulative_s, modeName(state.active_mode));
      resolved_mode = state.active_mode;
    }

    if (resolved_mode != state.active_mode) {
      const double since_switch_s = static_cast<double>(now_ns - state.last_switch_ns) / 1e9;
      // 回到 move 代表当前追击/防御条件已经解除，应该允许尽快释放姿态，
      // 否则会出现导航目标已经切回巡航，但下位机模式还被旧的 attack/defend 卡住几秒。
      const bool releasing_to_move = (resolved_mode == kMoveMode);
      // defend 属于保命姿态，不能被普通的切换冷却挡住，否则会出现血量已经跌破阈值，
      // 但 RViz / 下位机仍停留在 move 或 attack 的错觉。
      const bool entering_defend = (resolved_mode == kDefendMode);
      // 目标姿态合法，但仍要经过冷却时间校验，避免行为树在高频 tick 中反复抢切。
      if (!releasing_to_move && !entering_defend && since_switch_s < cooldown_s) {
        RCLCPP_INFO_THROTTLE(
          logger_, *node_->get_clock(), 1000,
          "Robot posture request '%s' blocked by cooldown: active=%s dt=%.2fs cooldown=%.2fs",
          modeName(resolved_mode), modeName(state.active_mode), since_switch_s, cooldown_s);
        resolved_mode = state.active_mode;
      }
    }
  }

  if (resolved_mode != state.active_mode) {
    const double previous_mode_used_s = state.cumulative_s[modeIndex(state.active_mode)];
    const double requested_mode_used_s = state.cumulative_s[modeIndex(resolved_mode)];
    RCLCPP_INFO(
      logger_,
      "Robot posture switched: %s -> %s (requested=%s, cooldown=%.1fs, used=%.1fs/%.1fs, next_used=%.1fs/%.1fs)",
      modeName(state.active_mode), modeName(resolved_mode), modeName(requested_mode), cooldown_s,
      previous_mode_used_s, max_cumulative_s, requested_mode_used_s, max_cumulative_s);
    // 只有真正发生切换时，才刷新当前姿态和切换时间戳。
    state.active_mode = resolved_mode;
    state.last_switch_ns = now_ns;
  }

  root_blackboard->set(kRobotModeRuntimeStateKey, state);
  root_blackboard->set("decision_active_robot_mode", std::string(modeName(state.active_mode)));
  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 3000,
    "[decision/mode] requested=%s active=%s game_progress=%u running=%d",
    modeName(requested_mode), modeName(state.active_mode), current_game_progress,
    static_cast<int>(match_running));
  publishModeVisualization(state.active_mode);
  return state.active_mode;
}

void PublishRobotModeAction::publishModeVisualization(uint8_t active_mode)
{
  if (!visualization_enabled_ || !visualization_publisher_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker text_marker;
  text_marker.header.frame_id = "base_link";
  // 对实车 TF 来说，marker 使用“当前时刻”时间戳容易比 map->base_link
  // 这条变换更靠前一点，RViz 会因此报 fixed frame 下无法变换。
  // 这里将 stamp 置零，显式要求 RViz 使用最新可用 TF。
  text_marker.header.stamp = builtin_interfaces::msg::Time();
  text_marker.ns = "robot_mode";
  text_marker.id = 0;
  text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::msg::Marker::ADD;
  text_marker.frame_locked = true;
  text_marker.pose.position.x = 0.0;
  text_marker.pose.position.y = 0.0;
  text_marker.pose.position.z = 1.6;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.35;
  text_marker.color.a = 1.0;
  text_marker.text = std::string("MODE: ") + modeName(active_mode);

  visualization_msgs::msg::Marker panel_marker;
  panel_marker.header = text_marker.header;
  panel_marker.ns = "robot_mode";
  panel_marker.id = 1;
  panel_marker.type = visualization_msgs::msg::Marker::CUBE;
  panel_marker.action = visualization_msgs::msg::Marker::ADD;
  panel_marker.frame_locked = true;
  panel_marker.pose.position.x = 0.0;
  panel_marker.pose.position.y = 0.0;
  panel_marker.pose.position.z = 1.3;
  panel_marker.pose.orientation.w = 1.0;
  panel_marker.scale.x = 0.7;
  panel_marker.scale.y = 0.18;
  panel_marker.scale.z = 0.08;
  panel_marker.color.a = 0.9;

  if (active_mode == kAttackMode) {
    text_marker.color.r = 1.0;
    text_marker.color.g = 0.95;
    text_marker.color.b = 0.95;
    panel_marker.color.r = 0.85;
    panel_marker.color.g = 0.15;
    panel_marker.color.b = 0.15;
  } else if (active_mode == kDefendMode) {
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    panel_marker.color.r = 0.15;
    panel_marker.color.g = 0.35;
    panel_marker.color.b = 0.95;
  } else {
    text_marker.color.r = 0.08;
    text_marker.color.g = 0.08;
    text_marker.color.b = 0.08;
    panel_marker.color.r = 0.15;
    panel_marker.color.g = 0.85;
    panel_marker.color.b = 0.25;
  }

  markers.markers.push_back(text_marker);
  markers.markers.push_back(panel_marker);
  visualization_publisher_->publish(markers);
}

}  // namespace ats_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(ats_sentry_behavior::PublishRobotModeAction, "PublishRobotMode");
