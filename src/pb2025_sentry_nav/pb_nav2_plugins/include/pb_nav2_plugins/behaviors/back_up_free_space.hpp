// Copyright 2024 Polaris Xia
 

#ifndef PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_behaviors/plugins/drive_on_heading.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using BackUpAction = nav2_msgs::action::BackUp;

namespace pb_nav2_behaviors
{

/**
 * @class pb_nav2_behaviors::BackUpFreeSpace
 * @brief An enhanced back_up action that move toward free space
 */
class BackUpFreeSpace : public nav2_behaviors::DriveOnHeading<nav2_msgs::action::BackUp>
{
public:
  BackUpFreeSpace() = default;

  // 恢复内部状态机。
  // 这里不是 Nav2 顶层 BT 的状态，而是恢复动作自己内部的执行阶段。
  // 这样做的目的是给恢复行为加“滞回”，避免因为瞬时障碍抖动就每拍重新规划或停启。
  enum class RecoveryExecutionState
  {
    PLANNING = 0,   // 正在根据 costmap 搜索一条恢复轨迹
    EXECUTING = 1,  // 已找到恢复轨迹，持续沿该轨迹执行
    BLOCKED = 2,    // 当前轨迹前缀连续若干拍被阻挡，等待冷却后重规划
  };

  // EscapePlan 不是全局路径，只是恢复行为在局部 costmap 上生成的一段短时逃逸轨迹。
  // 对全向舵轮来说，它更像一条“短走廊”，允许斜后退 / 侧后退，而不是死板纯后退。
  struct EscapePlan
  {
    bool valid = false;
    double heading = 0.0;                       // 恢复主方向，单位 rad
    double distance = 0.0;                      // 期望恢复距离，单位 m
    double score = 0.0;                         // 候选轨迹评分，越小越优
    geometry_msgs::msg::Point goal_point;       // 恢复轨迹终点
    std::vector<geometry_msgs::msg::Point> centerline;  // 恢复轨迹中心线采样点
  };

  /**
   * @brief Configuration of behavior action
   */
  void onConfigure() override;

  /**
   * @brief Cleanup server on lifecycle transition
   */
  void onCleanup() override;

  /**
   * @brief Initialization to run behavior
   * @param command Goal to execute
   * @return Status of behavior
   */
  nav2_behaviors::Status onRun(const std::shared_ptr<const BackUpAction::Goal> command) override;

  /**
   * @brief Loop function to run behavior
   * @return Status of behavior
   */
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  // 获取当前 costmap 快照。
  // 恢复动作不依赖“上一次的历史 costmap”，而是每次规划 / 重规划时获取一份最新地图。
  bool fetchCostmap(nav2_msgs::msg::Costmap & costmap);
  // 把当前位姿转换为 2D，便于做局部平面逃逸轨迹规划。
  geometry_msgs::msg::Pose2D poseToPose2D(const geometry_msgs::msg::PoseStamped & pose) const;
  // 根据当前位姿和目标恢复距离，在局部 costmap 中搜索一条最合适的恢复轨迹。
  // 搜索不是只看“一个方向”，而是遍历后向和侧后向的多个候选方向。
  bool planEscapeTrajectory(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    double target_distance, EscapePlan & best_plan);
  // 评估单个候选恢复方向。
  // 这里会沿着一条“有宽度的走廊”批量采样，而不是只检查一条线。
  bool evaluateCandidateTrajectory(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    double heading, double target_distance, EscapePlan & candidate) const;
  // 采样局部 costmap 指定位置的代价值。
  // 返回空表示超出地图范围，这类候选方向会直接判为不可用。
  std::optional<unsigned char> sampleCost(
    const nav2_msgs::msg::Costmap & costmap, double x, double y) const;
  // 执行阶段对当前恢复轨迹的前缀做连续前视检测。
  // 这一步用于判断“当前轨迹前方一小段是否仍可走”，而不是只看眼前一个离散点。
  bool isTrajectoryPrefixSafe(const geometry_msgs::msg::Pose2D & pose, double remaining_distance);
  // 计算机器人当前已经沿恢复主方向走了多远。
  // 这里使用对恢复方向的投影距离，而不是简单欧式距离，避免全向侧移时进度判断失真。
  double computeProgressAlongPlan(const geometry_msgs::msg::Pose2D & pose) const;
  // 根据剩余距离构造期望速度。
  // 会结合制动距离自动减速，避免冲过恢复终点。
  geometry_msgs::msg::Twist buildDesiredCommand(double remaining_distance) const;
  // 对期望速度做一阶低通和平移加减速限幅。
  // 这是保护舵轮电机、降低底盘高频抖动的关键环节。
  geometry_msgs::msg::Twist smoothCommand(
    const geometry_msgs::msg::Twist & desired_cmd, double dt);
  // 清空恢复行为的内部状态缓存。
  void resetExecutionState();
  // 当前轨迹被连续阻挡后，从机器人当前位置重新规划恢复轨迹。
  bool replanFromCurrentPose(
    const geometry_msgs::msg::PoseStamped & current_pose, double remaining_distance);
  // 发布恢复轨迹可视化，便于在 RViz 中观察恢复方向和终点。
  void visualizePlan(const geometry_msgs::msg::Pose2D & pose, const EscapePlan & plan);

  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>>
    marker_pub_;
  geometry_msgs::msg::Twist filtered_cmd_;
  geometry_msgs::msg::PoseStamped plan_start_pose_;
  EscapePlan active_plan_;
  RecoveryExecutionState execution_state_ = RecoveryExecutionState::PLANNING;
  std::optional<rclcpp::Time> last_cycle_time_;
  std::optional<rclcpp::Time> last_replan_time_;
  double command_distance_abs_ = 0.0;
  double command_speed_abs_ = 0.0;
  int blocked_cycles_ = 0;
  int clear_cycles_ = 0;
  int failed_replan_attempts_ = 0;
  double previous_plan_heading_ = 0.0;
  bool has_previous_plan_heading_ = false;

  // parameters
  std::string service_name_;
  double max_radius_;                  // 恢复搜索半径上限。大：更容易找到远处空隙；小：更保守。
  int max_allowed_cost_;              // 允许经过的最大 cost。小：更保守，离障远；大：更激进，可能贴墙。
  bool visualize_;                    // 是否发布恢复轨迹 marker。
  double search_half_span_deg_;       // 以车尾为中心的搜索半角。大：可尝试更侧向的退让。
  double search_angle_increment_deg_; // 候选方向角分辨率。小：更细致；大：计算更省。
  double trajectory_sample_step_;     // 沿恢复轨迹前进方向的采样间距。小：检测更细；大：更快。
  double corridor_half_width_;        // 恢复轨迹走廊半宽，近似代表底盘横向占用和安全裕量。
  double corridor_lateral_step_;      // 走廊横向采样间距。小：更严谨；大：更快。
  double heading_stickiness_weight_;  // 对上一条恢复方向的黏性权重。大：更稳；小：更灵活。
  double replanning_cooldown_s_;      // 两次重规划之间的最短间隔，防止每拍重规划。
  int blocked_enter_cycles_;          // 连续多少拍阻挡才进入 BLOCKED，形成进入滞回。
  int clear_exit_cycles_;             // 连续多少拍通畅才退出 BLOCKED，形成退出滞回。
  int max_replan_attempts_;           // 最大重规划次数，超过则判恢复失败。
  double speed_filter_tau_;           // 一阶低通时间常数。大：更平滑；小：更跟手。
  double translational_acc_limit_;    // 恢复阶段平移加速度上限。
  double translational_decel_limit_;  // 恢复阶段平移减速度上限。
  double minimum_speed_xy_;           // 恢复阶段的最小平移速度，避免末段反复抖动。
  double goal_tolerance_;             // 恢复到终点的距离容差。
  double monitor_lookahead_distance_; // 恢复执行时前视检测长度。大：更早预判；小：更激进。
  bool enable_full_circle_fallback_;  // 侧后退都失败时，是否放开到全方向搜索。
};

}  // namespace pb_nav2_behaviors

#endif  // PB_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
