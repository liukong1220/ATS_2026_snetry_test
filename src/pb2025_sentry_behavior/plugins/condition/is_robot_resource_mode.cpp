#include "pb2025_sentry_behavior/plugins/condition/is_robot_resource_mode.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace pb2025_sentry_behavior
{

namespace
{

constexpr char kResourceRuntimeStateKey[] = "decision_resource_runtime_state";
constexpr char kResourceModeBlackboardKey[] = "decision_resource_mode";

enum class ResourceMode
{
  kUnknown = 0,
  kEngage,
  kResupply,
  kDefend,
};

struct ResourceRuntimeState
{
  bool initialized = false;
  ResourceMode latched_mode = ResourceMode::kUnknown;
};

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

const char * modeName(const ResourceMode mode)
{
  switch (mode) {
    case ResourceMode::kEngage:
      return "engage";
    case ResourceMode::kResupply:
      return "resupply";
    case ResourceMode::kDefend:
      return "defend";
    case ResourceMode::kUnknown:
    default:
      return "unknown";
  }
}

ResourceMode parseExpectedMode(const std::string & raw_mode)
{
  const auto mode = normalizeToken(raw_mode);
  if (mode == "engage") {
    return ResourceMode::kEngage;
  }
  if (mode == "resupply" || mode == "supply" || mode == "safe") {
    return ResourceMode::kResupply;
  }
  if (mode == "defend" || mode == "defense" || mode == "retreat") {
    return ResourceMode::kDefend;
  }
  return ResourceMode::kUnknown;
}

}  // namespace

IsRobotResourceModeCondition::IsRobotResourceModeCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(
    name, std::bind(&IsRobotResourceModeCondition::tickCondition, this), config),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
  node_->get_parameter("decision.resource_policy.defend_enter_hp", defend_enter_hp_);
  node_->get_parameter("decision.resource_policy.defend_exit_hp", defend_exit_hp_);
  node_->get_parameter("decision.resource_policy.resupply_enter_hp", resupply_enter_hp_);
  node_->get_parameter("decision.resource_policy.resupply_exit_hp", resupply_exit_hp_);
  node_->get_parameter("decision.resource_policy.resupply_enter_ammo", resupply_enter_ammo_);
  node_->get_parameter("decision.resource_policy.resupply_exit_ammo", resupply_exit_ammo_);
}

BT::NodeStatus IsRobotResourceModeCondition::tickCondition()
{
  std::string expected_mode_raw = "engage";
  if (!getInput("state", expected_mode_raw)) {
    RCLCPP_ERROR(logger_, "IsRobotResourceMode did not receive state input");
    return BT::NodeStatus::FAILURE;
  }

  const auto expected_mode = parseExpectedMode(expected_mode_raw);
  if (expected_mode == ResourceMode::kUnknown) {
    RCLCPP_ERROR(
      logger_, "IsRobotResourceMode received unsupported state='%s'",
      expected_mode_raw.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto root_blackboard = config().blackboard->rootBlackboard();
  if (root_blackboard == nullptr) {
    RCLCPP_ERROR(logger_, "BehaviorTree root blackboard is not available");
    return BT::NodeStatus::FAILURE;
  }

  auto robot_status = getInput<pb_rm_interfaces::msg::RobotStatus>("robot_status");
  if (!robot_status) {
    // 仿真若没有发布假裁判输入，则不强行阻断主树，让后续 simulation/referee 分支继续接管。
    root_blackboard->set<std::string>(kResourceModeBlackboardKey, "unknown");
    RCLCPP_INFO_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Decision resource mode unresolved: referee/robot_status is unavailable");
    return BT::NodeStatus::FAILURE;
  }

  ResourceRuntimeState runtime_state;
  if (!root_blackboard->get(kResourceRuntimeStateKey, runtime_state) || !runtime_state.initialized) {
    runtime_state.initialized = true;
    runtime_state.latched_mode = ResourceMode::kEngage;
  }

  const int current_hp = static_cast<int>(robot_status->current_hp);
  const int current_ammo = static_cast<int>(robot_status->projectile_allowance_17mm);
  const int defend_exit_hp = std::max(defend_exit_hp_, defend_enter_hp_);
  const int resupply_exit_hp = std::max(resupply_exit_hp_, resupply_enter_hp_);
  const int resupply_exit_ammo = std::max(resupply_exit_ammo_, resupply_enter_ammo_);

  ResourceMode resolved_mode = runtime_state.latched_mode;

  // 决策优先级：
  // 1. defend  : 血量已经低到必须先保命。
  // 2. resupply: 虽未到极低血量，但已不适合继续追击，应回补给安全点。
  // 3. engage  : 血量/弹量都在健康区间，允许巡航和视觉接管。
  switch (runtime_state.latched_mode) {
    case ResourceMode::kDefend:
      if (current_hp <= defend_exit_hp) {
        resolved_mode = ResourceMode::kDefend;
      } else if (current_hp <= resupply_exit_hp || current_ammo <= resupply_exit_ammo) {
        resolved_mode = ResourceMode::kResupply;
      } else {
        resolved_mode = ResourceMode::kEngage;
      }
      break;

    case ResourceMode::kResupply:
      if (current_hp <= defend_enter_hp_) {
        resolved_mode = ResourceMode::kDefend;
      } else if (current_hp <= resupply_exit_hp || current_ammo <= resupply_exit_ammo) {
        resolved_mode = ResourceMode::kResupply;
      } else {
        resolved_mode = ResourceMode::kEngage;
      }
      break;

    case ResourceMode::kUnknown:
    case ResourceMode::kEngage:
    default:
      if (current_hp <= defend_enter_hp_) {
        resolved_mode = ResourceMode::kDefend;
      } else if (current_hp <= resupply_enter_hp_ || current_ammo <= resupply_enter_ammo_) {
        resolved_mode = ResourceMode::kResupply;
      } else {
        resolved_mode = ResourceMode::kEngage;
      }
      break;
  }

  if (resolved_mode != runtime_state.latched_mode) {
    RCLCPP_INFO(
      logger_,
      "Decision resource mode switched: %s -> %s (hp=%d, ammo=%d)",
      modeName(runtime_state.latched_mode), modeName(resolved_mode), current_hp, current_ammo);
    runtime_state.latched_mode = resolved_mode;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *node_->get_clock(), 2000,
    "Decision resource mode=%s expected=%s hp=%d ammo=%d",
    modeName(resolved_mode), modeName(expected_mode), current_hp, current_ammo);

  root_blackboard->set(kResourceRuntimeStateKey, runtime_state);
  root_blackboard->set<std::string>(kResourceModeBlackboardKey, modeName(resolved_mode));

  return resolved_mode == expected_mode ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsRobotResourceModeCondition::providedPorts()
{
  return {
    BT::InputPort<pb_rm_interfaces::msg::RobotStatus>(
      "robot_status", "{@referee_robotStatus}",
      "裁判系统 RobotStatus，读取 current_hp 与 projectile_allowance_17mm"),
    BT::InputPort<std::string>(
      "state", "engage",
      "期望资源状态：engage / resupply / defend")};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsRobotResourceModeCondition>(
    "IsRobotResourceMode");
}
