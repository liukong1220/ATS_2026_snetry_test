#include "standard_robot_pp_ros2/2025_robot_decision.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

std::string formatIndexList(const std::vector<size_t> & indices)
{
  std::ostringstream oss;
  oss << "[";
  for (size_t i = 0; i < indices.size(); ++i) {
    if (i != 0) {
      oss << ", ";
    }
    oss << indices[i];
  }
  oss << "]";
  return oss.str();
}

}  // namespace

namespace standard_robot_pp_ros2
{

RobotDecisionNode::RobotDecisionNode(const rclcpp::NodeOptions & options)
: Node("RobotDecisionNode", options),
  nav2_client_(nullptr),  // 按类声明顺序初始化
  current_goal_handle_(nullptr),
  rmul_goal_point_(),
  params_(),
  current_feedback_(std::make_shared<NavigateToPose::Feedback>()),
  last_nav_result_(rclcpp_action::ResultCode::UNKNOWN),
  robot_move_(false),
  last_goal_send_time_(0, 0, RCL_ROS_TIME),
  stop_release_time_(0, 0, RCL_ROS_TIME)
{
  RCLCPP_INFO(get_logger(), "Start RobotDecisionNode!");

  // 参数声明部分修改为完整命名空间路径
  auto declare_ns_param = [this](const std::string & name, auto && default_val) {
    return this->declare_parameter<std::decay_t<decltype(default_val)>>(
      name, std::forward<decltype(default_val)>(default_val));
  };

  // 加载动作服务名称参数
  params_.config.nav2_action_server =
    declare_ns_param("decision_config.nav2_action_server", std::string("/navigate_to_pose"));

  // 目标点参数
  auto goal_x =
    declare_ns_param("goal_points.x", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  auto goal_y =
    declare_ns_param("goal_points.y", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  auto goal_z =
    declare_ns_param("goal_points.z", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  // 添加调试日志输出
  if (goal_x.size() > 5 && goal_y.size() > 5 && goal_z.size() > 5) {
    RCLCPP_INFO(this->get_logger(), "Loaded goal_x: %f", goal_x[5]);
    RCLCPP_INFO(this->get_logger(), "Loaded goal_y: %f", goal_y[5]);
    RCLCPP_INFO(this->get_logger(), "Loaded goal_z: %f", goal_z[5]);
  }

  // 验证坐标维度一致性
  if (goal_x.size() != goal_y.size() || goal_x.size() != goal_z.size()) {
    RCLCPP_FATAL(
      this->get_logger(), "三维坐标参数长度不匹配！x:%zu vs y:%zu vs z:%zu", goal_x.size(),
      goal_y.size(), goal_z.size());
    throw std::runtime_error("三维坐标参数长度不匹配");
  }

  // 构建三维目标点数组
  rmul_goal_point_.clear();
  for (size_t i = 0; i < goal_x.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = goal_x[i];
    p.y = goal_y[i];
    p.z = goal_z[i];  // 使用三维坐标
    rmul_goal_point_.push_back(p);
  }

  // 检查目标点数量（根据实际需求保留或修改）
  // if (rmul_goal_point_.size() < 4) {
  //     RCLCPP_FATAL(get_logger(),
  //             "需要至少4个三维目标点配置，当前数量: %zu",
  //             rmul_goal_point_.size());
  //     throw std::runtime_error("Invalid goal points configuration");
  // }

  // 调试日志输出
  for (const auto & point : rmul_goal_point_) {
    RCLCPP_INFO(
      this->get_logger(),
      "三维路径点坐标: (%.2f, %.2f, %.2f)",  // ✅ 补全括号和分号
      point.x, point.y,
      point.z);  // 补全闭合括号
  }

  const int supply_safe_point_index = declare_ns_param("point_roles.supply_safe_point_index", 0);
  const auto patrol_indices_raw =
    declare_ns_param("point_roles.patrol_indices", std::vector<int64_t>{1, 2, 3});
  const auto low_hp_candidate_indices_raw =
    declare_ns_param("point_roles.low_hp_candidate_indices", std::vector<int64_t>{1, 2, 3});
  const int critical_time_target_index =
    declare_ns_param("point_roles.critical_time_target_index", 1);

  auto validate_single_index = [this](int index_value, const std::string & param_name) -> size_t {
    if (index_value < 0 || static_cast<size_t>(index_value) >= rmul_goal_point_.size()) {
      RCLCPP_FATAL(
        get_logger(), "参数 %s 非法，索引值 %d 超出目标点范围 [0, %zu)", param_name.c_str(),
        index_value, rmul_goal_point_.size());
      throw std::runtime_error("point role index out of range");
    }
    return static_cast<size_t>(index_value);
  };

  auto validate_index_list =
    [this](const std::vector<int64_t> & raw_indices, const std::string & param_name) {
      if (raw_indices.empty()) {
        RCLCPP_FATAL(get_logger(), "参数 %s 不能为空", param_name.c_str());
        throw std::runtime_error("point role index list is empty");
      }

      std::vector<size_t> validated_indices;
      validated_indices.reserve(raw_indices.size());
      for (const auto raw_index : raw_indices) {
        if (raw_index < 0 || static_cast<size_t>(raw_index) >= rmul_goal_point_.size()) {
          RCLCPP_FATAL(
            get_logger(), "参数 %s 非法，索引值 %ld 超出目标点范围 [0, %zu)", param_name.c_str(),
            raw_index, rmul_goal_point_.size());
          throw std::runtime_error("point role index list out of range");
        }
        validated_indices.push_back(static_cast<size_t>(raw_index));
      }
      return validated_indices;
    };

  supply_safe_point_index_ =
    validate_single_index(supply_safe_point_index, "point_roles.supply_safe_point_index");
  patrol_indices_ = validate_index_list(patrol_indices_raw, "point_roles.patrol_indices");
  low_hp_candidate_indices_ =
    validate_index_list(low_hp_candidate_indices_raw, "point_roles.low_hp_candidate_indices");
  critical_time_target_index_ =
    validate_single_index(critical_time_target_index, "point_roles.critical_time_target_index");
  patrol_cursor_ = 0;
  patrol_direction_ = 1;

  // 加载时间阈值参数
  params_.time.abundant = declare_ns_param("time_thresholds.abundant", 420);
  params_.time.normal = declare_ns_param("time_thresholds.normal", 360);
  params_.time.tense = declare_ns_param("time_thresholds.tense", 180);
  params_.time.critical = declare_ns_param("time_thresholds.critical", 60);

  // 加载血量阈值参数
  params_.hp.abundant.high = declare_ns_param("hp_thresholds.abundant.high", 300);
  params_.hp.abundant.medium = declare_ns_param("hp_thresholds.abundant.medium", 200);
  params_.hp.abundant.low = declare_ns_param("hp_thresholds.abundant.low", 150);

  params_.hp.normal.high = declare_ns_param("hp_thresholds.normal.high", 300);
  params_.hp.normal.medium = declare_ns_param("hp_thresholds.normal.medium", 200);
  params_.hp.normal.low = declare_ns_param("hp_thresholds.normal.low", 150);

  params_.hp.tense.high = declare_ns_param("hp_thresholds.tense.high", 300);
  params_.hp.tense.medium = declare_ns_param("hp_thresholds.tense.medium", 200);
  params_.hp.tense.low = declare_ns_param("hp_thresholds.tense.low", 150);

  params_.hp.critical.high = declare_ns_param("hp_thresholds.critical.high", 300);
  params_.hp.critical.medium = declare_ns_param("hp_thresholds.critical.medium", 200);
  params_.hp.critical.low = declare_ns_param("hp_thresholds.critical.low", 100);

  // 加载决策配置参数
  params_.config.path_tolerance = declare_ns_param("decision_config.path_tolerance", 0.1);
  params_.config.decision_period_ms = declare_ns_param("decision_config.decision_period_ms", 100);
  params_.config.goal_republish_interval_s =
    declare_ns_param("decision_config.goal_republish_interval_s", 1.0);
  params_.config.goal_position_tolerance =
    declare_ns_param("decision_config.goal_position_tolerance", 0.05);
  params_.config.waypoint_stop_duration_s =
    declare_ns_param("decision_config.waypoint_stop_duration_s", 0.0);
  params_.config.decision_period_ms = std::clamp(params_.config.decision_period_ms, 20, 1000);

  // 调试输出结构体参数值
  RCLCPP_INFO(get_logger(), "[结构体参数验证]");
  RCLCPP_INFO(get_logger(), "时间阈值-充足: %d", params_.time.abundant);
  RCLCPP_INFO(get_logger(), "血量阈值-中等: %d", params_.hp.critical.medium);
  RCLCPP_INFO(get_logger(), "路径容差: %.2f", params_.config.path_tolerance);
  RCLCPP_INFO(get_logger(), "决策周期: %d ms", params_.config.decision_period_ms);
  RCLCPP_INFO(
    get_logger(), "补给安全点索引: %zu 坐标: (%.2f, %.2f, %.2f)", supply_safe_point_index_,
    rmul_goal_point_[supply_safe_point_index_].x, rmul_goal_point_[supply_safe_point_index_].y,
    rmul_goal_point_[supply_safe_point_index_].z);
  RCLCPP_INFO(get_logger(), "巡逻点列表: %s", formatIndexList(patrol_indices_).c_str());
  RCLCPP_INFO(
    get_logger(), "低血量候选列表: %s", formatIndexList(low_hp_candidate_indices_).c_str());
  RCLCPP_INFO(
    get_logger(), "时间严重紧张目标索引: %zu 坐标: (%.2f, %.2f, %.2f)", critical_time_target_index_,
    rmul_goal_point_[critical_time_target_index_].x,
    rmul_goal_point_[critical_time_target_index_].y,
    rmul_goal_point_[critical_time_target_index_].z);

  // 订阅比赛状态、RFID状态和机器人状态消息
  game_status_sub_ = create_subscription<pb_rm_interfaces::msg::GameStatus>(
    "referee/game_status", 10,
    std::bind(&RobotDecisionNode::gameStatusCallback, this, std::placeholders::_1));

  rfid_status_sub_ = create_subscription<pb_rm_interfaces::msg::RfidStatus>(
    "referee/rfid_status", 10,
    std::bind(&RobotDecisionNode::rfidStatusCallback, this, std::placeholders::_1));

  robot_status_sub_ = create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    "referee/robot_status", 10,
    std::bind(&RobotDecisionNode::robotStatusCallback, this, std::placeholders::_1));

  stop_flag_pub_ = create_publisher<std_msgs::msg::Bool>("stop_flag", 10);  // 发布停止标志位

  // 初始化Action客户端
  nav2_client_ =
    rclcpp_action::create_client<NavigateToPose>(this, params_.config.nav2_action_server);

  // 动作服务器连接检测
  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_FATAL(
      get_logger(), "Action server %s not available", params_.config.nav2_action_server.c_str());
    throw std::runtime_error("Navigation action server not available");
  }

  // 创建定时器，定期执行决策逻辑        // 创建定时器，定期执行决策逻辑
  timer_ =
    create_wall_timer(std::chrono::milliseconds(params_.config.decision_period_ms), [this]() {
      std::lock_guard<std::mutex> lock(nav_mutex_);  // 添加互斥锁
      try {
        executeDecisionLogic();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "决策异常: %s", e.what());
      }
    });
}

void RobotDecisionNode::gameStatusCallback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg)
{
  game_status_msg_ = *msg;
  if (last_game_progress_ != game_status_msg_.game_progress) {
    last_game_progress_ = game_status_msg_.game_progress;
    RCLCPP_INFO(get_logger(), "比赛状态切换: %d", game_status_msg_.game_progress);
  }
}

void RobotDecisionNode::rfidStatusCallback(const pb_rm_interfaces::msg::RfidStatus::SharedPtr msg)
{
  rfid_status_msg_ = *msg;
}

void RobotDecisionNode::robotStatusCallback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg)
{
  robot_status_msg_ = *msg;
}

void RobotDecisionNode::executeDecisionLogic()
{
  // //判断比赛是否开始
  // if (game_status_msg_.game_progress == 3)
  // {
  //     robot_move_ = false;

  // }
  // else if (game_status_msg_.game_progress == 4)
  // {
  //     robot_move_ = true;
  // }
  // else
  // {
  //     robot_move_ = false;
  // }

  // 决策函数
  robotDecision();
}

void RobotDecisionNode::sendNavigationGoal(float x, float y, float z, float yaw)
{
  // 检查动作服务器状态
  if (!nav2_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), "动作服务器不可用");
    return;
  }

  const auto now = this->now();
  const bool same_goal =
    has_last_goal_ &&
    std::hypot(last_goal_point_.x - x, last_goal_point_.y - y) <=
      params_.config.goal_position_tolerance &&
    std::fabs(last_goal_point_.z - z) <= params_.config.goal_position_tolerance &&
    std::fabs(last_goal_yaw_ - yaw) <= 0.05;

  if (same_goal && (goal_request_pending_ || current_goal_handle_)) {
    return;
  }

  if (
    same_goal &&
    (now - last_goal_send_time_).seconds() < params_.config.goal_republish_interval_s) {
    return;
  }

  // 取消当前正在执行的目标（非阻塞方式）
  if (!same_goal && current_goal_handle_) {
    auto future_cancel = nav2_client_->async_cancel_goal(current_goal_handle_);
    (void)future_cancel;
    RCLCPP_DEBUG(get_logger(), "目标已变化，发送取消请求");
    current_goal_handle_.reset();
  }

  // 构建动作目标
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = "map";

  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  // 设置发送选项
  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // 目标响应回调
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNavigate::SharedPtr & goal_handle) {
      goal_request_pending_ = false;
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "目标被服务器拒绝");
      } else {
        current_goal_handle_ = goal_handle;  // 关键修改：存储目标句柄
        RCLCPP_INFO(
          get_logger(), "目标已接受，ID: %s",
          rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
      }
    };

  // 在send_goal_options添加反馈回调
  send_goal_options.feedback_callback =
    [this](
      GoalHandleNavigate::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      // current_feedback_ = feedback; // 直接替换指针
      has_feedback_ = true;
      current_feedback_->current_pose = feedback->current_pose;  // 复制数据而非指针
      RCLCPP_DEBUG(get_logger(), "收到导航反馈");
    };

  // 结果回调
  send_goal_options.result_callback = [this](const GoalHandleNavigate::WrappedResult & result) {
    last_nav_result_ = result.code;  // 记录导航结果
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "导航成功抵达目标点");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "导航被服务器中止");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_DEBUG(get_logger(), "导航任务被取消");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "未知导航结果");
        break;
    }
    current_goal_handle_.reset();  // 重置目标句柄
    goal_request_pending_ = false;
  };

  // 异步发送目标
  // 处理发送结果
  try {
    // 异步发送目标（移除spin_until_future_complete）
    auto future_goal_handle = nav2_client_->async_send_goal(goal_msg, send_goal_options);
    (void)future_goal_handle;
    has_last_goal_ = true;
    goal_request_pending_ = true;
    last_goal_point_.x = x;
    last_goal_point_.y = y;
    last_goal_point_.z = z;
    last_goal_yaw_ = yaw;
    last_goal_send_time_ = now;
    RCLCPP_DEBUG(get_logger(), "已异步发送导航目标");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "发送目标异常: %s", e.what());
  }
}

int RobotDecisionNode::getGameTimeState()
{
  RCLCPP_DEBUG(get_logger(), "当前游戏时间状态: %d", game_status_msg_.stage_remain_time);
  if (
    game_status_msg_.stage_remain_time > params_.time.normal &&
    game_status_msg_.stage_remain_time <= params_.time.abundant) {
    return 1;  // 时间充足
  } else if (
    game_status_msg_.stage_remain_time > params_.time.tense &&
    game_status_msg_.stage_remain_time <= params_.time.normal) {
    return 2;  // 时间常规
  } else if (
    game_status_msg_.stage_remain_time > params_.time.critical &&
    game_status_msg_.stage_remain_time <= params_.time.tense) {
    return 3;  // 时间紧张
  } else if (
    game_status_msg_.stage_remain_time > 0 &&
    game_status_msg_.stage_remain_time <= params_.time.critical) {
    return 4;  // 时间严重紧张
  } else {
    RCLCPP_WARN(get_logger(), "未知时间状态: %d", game_status_msg_.stage_remain_time);
    return -1;
  }
  return 0;
}

void RobotDecisionNode::robotDecision()
{
  updateStopState();

  // if (!robot_move_) {
  //     if (!stop_hold_active_) {
  //         publishStopFlag(false);
  //     }
  //     RCLCPP_WARN_THROTTLE(
  //         get_logger(), *this->get_clock(), 1000, "移动禁止状态");
  //     return;
  // }

  // if (!stop_hold_active_) {
  //     publishStopFlag(false);
  // }

  // const int current_game_state = getGameTimeState();
  const int current_hp = robot_status_msg_.current_hp;
  const double tolerance = params_.config.path_tolerance;

  if (current_hp > params_.hp.normal.medium && current_hp <= params_.hp.normal.high) {
    handleNormalHpBehavior(tolerance);
  } else if (current_hp > params_.hp.normal.low && current_hp <= params_.hp.normal.medium) {
    handleLowHpBehavior(tolerance);
  } else {
    handleCriticalHpBehavior(tolerance);
  }
}

// 辅助函数实现

void RobotDecisionNode::handleNormalHpBehavior(double tolerance)
{
  low_hp_target_index_ = std::numeric_limits<size_t>::max();

  if (patrol_indices_.empty()) {
    RCLCPP_WARN(get_logger(), "巡逻点列表为空，无法执行常规巡逻");
    return;
  }

  if (patrol_cursor_ >= patrol_indices_.size()) {
    patrol_cursor_ = 0;
  }

  const size_t target_index = patrol_indices_[patrol_cursor_];
  const auto & target = rmul_goal_point_[target_index];

  if (checkDistance(target, tolerance)) {
    triggerStopSequence();

    if (patrol_indices_.size() > 1) {
      int next_cursor = static_cast<int>(patrol_cursor_) + patrol_direction_;

      if (next_cursor >= static_cast<int>(patrol_indices_.size())) {
        patrol_direction_ = -1;
        next_cursor = static_cast<int>(patrol_indices_.size()) - 2;
      } else if (next_cursor < 0) {
        patrol_direction_ = 1;
        next_cursor = 1;
      }
      patrol_cursor_ = static_cast<size_t>(next_cursor);
    }

    RCLCPP_INFO(
      get_logger(), "更新巡逻游标至: %zu, 当前目标点索引: %zu", patrol_cursor_,
      patrol_indices_[patrol_cursor_]);
  }

  const auto & next_target = rmul_goal_point_[patrol_indices_[patrol_cursor_]];
  sendNavigationGoal(next_target.x, next_target.y, next_target.z);
}

void RobotDecisionNode::handleLowHpBehavior(double tolerance)
{
  if (!has_feedback_) {
    RCLCPP_WARN(get_logger(), "等待导航反馈数据...");
    return;
  }

  const auto & current_position = current_feedback_->current_pose.pose.position;

  if (low_hp_target_index_ >= rmul_goal_point_.size()) {
    low_hp_target_index_ = findNearestPoint(current_position, low_hp_candidate_indices_);
  }

  if (low_hp_target_index_ >= rmul_goal_point_.size()) {
    RCLCPP_ERROR(get_logger(), "无效退防点索引: %zu", low_hp_target_index_);
    low_hp_target_index_ = std::numeric_limits<size_t>::max();
    return;
  }

  const auto & target_point = rmul_goal_point_[low_hp_target_index_];
  if (checkDistance(target_point, tolerance)) {
    RCLCPP_INFO(get_logger(), "已到达退防点[%zu]", low_hp_target_index_);
    triggerStopSequence();
    low_hp_target_index_ = std::numeric_limits<size_t>::max();
    return;
  }

  sendNavigationGoal(target_point.x, target_point.y, target_point.z);
}

// 修改后的危急血处理函数
void RobotDecisionNode::handleCriticalHpBehavior(double tolerance)
{
  RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000, "血量危急，回到安全点");

  // 发布目标点
  if (rmul_goal_point_.empty()) {
    RCLCPP_ERROR(get_logger(), "目标点列表为空，无法回安全点");
    return;
  }
  const auto & target = rmul_goal_point_[supply_safe_point_index_];

  low_hp_target_index_ = std::numeric_limits<size_t>::max();
  if (checkDistance(target, tolerance)) {
    triggerStopSequence();
    return;
  }

  sendNavigationGoal(target.x, target.y, target.z);
}

bool RobotDecisionNode::checkDistance(const geometry_msgs::msg::Point & target, double tolerance)
{
  if (!has_feedback_ || !current_feedback_) {
    RCLCPP_WARN(get_logger(), "等待导航反馈数据...");
    return false;
  }

  const auto & current_pose = current_feedback_->current_pose.pose.position;
  const double dx = current_pose.x - target.x;
  const double dy = current_pose.y - target.y;
  return (sqrt(dx * dx + dy * dy) <= tolerance);
}

size_t RobotDecisionNode::findNearestPoint(
  const geometry_msgs::msg::Point & reference_point, const std::vector<size_t> & candidate_indices)
{
  // 安全检查
  if (rmul_goal_point_.empty()) {
    RCLCPP_ERROR(get_logger(), "目标点列表为空！");
    return 0;
  }

  if (!has_feedback_ || !current_feedback_) {
    RCLCPP_WARN(get_logger(), "导航反馈异常，返回补给安全点");
    return supply_safe_point_index_;
  }

  if (candidate_indices.empty()) {
    RCLCPP_ERROR(get_logger(), "候选索引列表为空，默认返回补给安全点");
    return supply_safe_point_index_;
  }

  // 初始化最近点索引和最小距离
  size_t nearest_index = candidate_indices.front();
  double min_distance = std::numeric_limits<double>::max();

  for (const auto candidate_index : candidate_indices) {
    const double dx = reference_point.x - rmul_goal_point_[candidate_index].x;
    const double dy = reference_point.y - rmul_goal_point_[candidate_index].y;
    const double distance = dx * dx + dy * dy;

    if (distance < min_distance) {
      min_distance = distance;
      nearest_index = candidate_index;
    }
  }
  RCLCPP_DEBUG(
    get_logger(), "参考点: (%.2f,%.2f) 低血量候选列表: %s", reference_point.x, reference_point.y,
    formatIndexList(candidate_indices).c_str());

  RCLCPP_DEBUG(get_logger(), "最近退防点索引: %zu (距离: %.2f)", nearest_index, sqrt(min_distance));
  return nearest_index;
}

void RobotDecisionNode::publishStopFlag(bool stop)
{
  if (!stop_flag_pub_) {
    RCLCPP_WARN(get_logger(), "停止标志发布器未初始化");
    return;
  }
  if (stop_flag_state_ == stop) {
    return;
  }
  auto stop_msg = std_msgs::msg::Bool();
  stop_msg.data = stop;
  stop_flag_pub_->publish(stop_msg);
  stop_flag_state_ = stop;
}

void RobotDecisionNode::updateStopState()
{
  if (!stop_hold_active_) {
    return;
  }
  if (this->now() >= stop_release_time_) {
    publishStopFlag(false);
    stop_hold_active_ = false;
  }
}

void RobotDecisionNode::triggerStopSequence()
{
  if (params_.config.waypoint_stop_duration_s <= 0.0) {
    return;
  }
  publishStopFlag(true);
  stop_hold_active_ = true;
  stop_release_time_ =
    this->now() + rclcpp::Duration::from_seconds(params_.config.waypoint_stop_duration_s);
}

void RobotDecisionNode::logDecisionState(int hp, size_t index)
{
  RCLCPP_INFO(get_logger(), "[状态] HP: %d, 移动: %s", hp, robot_move_ ? "允许" : "禁止");
  RCLCPP_INFO(
    get_logger(), "[目标点] 索引: %zu 坐标: (%.2f, %.2f)", index, rmul_goal_point_[index].x,
    rmul_goal_point_[index].y);
}

}  // namespace standard_robot_pp_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(standard_robot_pp_ros2::RobotDecisionNode)
