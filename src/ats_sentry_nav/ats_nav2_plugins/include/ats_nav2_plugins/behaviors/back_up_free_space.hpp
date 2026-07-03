// Copyright 2024 Polaris Xia
 

#ifndef ATS_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
#define ATS_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_

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

namespace ats_nav2_behaviors
{

/**
 * @class ats_nav2_behaviors::BackUpFreeSpace
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

  // 当前恢复轨迹来自哪种规划分支：
  // 1. CORRIDOR_PRIMARY：主走廊搜索成功
  // 2. CENTROID_FALLBACK：主走廊失败后退化到自由空间重心方向
  enum class PlanSource
  {
    CORRIDOR_PRIMARY = 0,
    CENTROID_FALLBACK = 1,
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
  // 当走廊搜索失败时，退化为“自由空间重心”方向估计，给恢复行为一个最后的逃逸方向。
  bool planCentroidFallbackTrajectory(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    double target_distance, EscapePlan & fallback_plan) const;
  // 评估单个候选恢复方向。
  // 这里会沿着一条“有宽度的走廊”批量采样，而不是只检查一条线。
  bool evaluateCandidateTrajectory(
    const nav2_msgs::msg::Costmap & costmap, const geometry_msgs::msg::Pose2D & pose,
    double heading, double target_distance, EscapePlan & candidate) const;
  // 采样局部 costmap 指定位置的代价值。
  // 返回空表示超出地图范围，这类候选方向会直接判为不可用。
  std::optional<unsigned char> sampleCost(
    const nav2_msgs::msg::Costmap & costmap, double x, double y) const;
  // 计算恢复轨迹中心线的平均 cost，用于把“勉强能走的窄缝”自动降速。
  double computePlanAverageCost(const nav2_msgs::msg::Costmap & costmap, const EscapePlan & plan) const;
  // 计算当前恢复轨迹前方还能安全通行多远。
  // 这是第二阶段“动态障碍简单速度预测”的观测量基础。
  double computeSafePrefixDistance(
    const geometry_msgs::msg::Pose2D & pose, double remaining_distance) const;
  // 执行阶段对当前恢复轨迹的前缀做连续前视检测。
  // 这一步用于判断“当前轨迹前方一小段是否仍可走”，而不是只看眼前一个离散点。
  bool isTrajectoryPrefixSafe(const geometry_msgs::msg::Pose2D & pose, double remaining_distance);
  // 计算机器人当前已经沿当前恢复主方向走了多远。
  // 这里使用对恢复方向的投影距离，而不是简单欧式距离，避免全向侧移时进度判断失真。
  double computeSegmentProgress(const geometry_msgs::msg::Pose2D & pose) const;
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
    const geometry_msgs::msg::PoseStamped & current_pose, double remaining_total_distance,
    double total_distance_traveled);
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
  std::optional<double> last_safe_prefix_distance_;
  double command_distance_abs_ = 0.0;
  double command_speed_abs_ = 0.0;
  double completed_distance_before_plan_ = 0.0;
  double estimated_prefix_rate_ = 0.0;
  int blocked_cycles_ = 0;
  int clear_cycles_ = 0;
  int failed_replan_attempts_ = 0;
  double previous_plan_heading_ = 0.0;
  bool has_previous_plan_heading_ = false;
  double active_plan_average_cost_ = 0.0;
  PlanSource active_plan_source_ = PlanSource::CORRIDOR_PRIMARY;

  // parameters
  std::string service_name_;
  double max_radius_;                  // 恢复搜索半径上限。大：更容易找到远处空隙；小：更保守。
  int max_allowed_cost_;              // 允许经过的最大 cost。小：更保守，离障远；大：更激进，可能贴墙。
  bool visualize_;                    // 是否发布恢复轨迹 marker。
  double search_half_span_deg_;       // 以车尾为中心的搜索半角。大：可尝试更侧向的退让。
  double search_angle_increment_deg_; // 候选方向角分辨率。小：更细致；大：计算更省。
  double trajectory_sample_step_;     // 兼容旧参数：若未分层配置，则作为默认采样间距。
  double near_sample_step_;           // 近距离纵向采样步长。近处更密，有利于防止贴近障碍时漏检。
  double far_sample_step_;            // 远距离纵向采样步长。远处更疏，节约算力。
  double layered_sampling_split_distance_;  // 近/远分层采样切换距离。
  double corridor_half_width_;        // 恢复轨迹走廊半宽，近似代表底盘横向占用和安全裕量。
  double corridor_lateral_step_;      // 近距离走廊横向采样间距。小：更严谨；大：更快。
  double far_corridor_lateral_step_;  // 远距离走廊横向采样间距。可适当更粗，节省算力。
  double minimum_release_distance_;   // 允许“分段放行”的最短安全段长度。太小会导致原地碎步抖动。
  bool dynamic_obstacle_prediction_enabled_;  // 是否开启第二阶段的动态障碍前沿速度预测。
  double prediction_horizon_s_;               // 预测时间窗。大：更早预判；小：更保守。
  double prefix_velocity_alpha_;              // 前沿速度估计低通系数。大：更灵敏；小：更稳。
  double predictive_block_margin_;            // 预测后仍需保留的最小安全前缀余量。
  double heading_stickiness_weight_;  // 对上一条恢复方向的黏性权重。大：更稳；小：更灵活。
  double replanning_cooldown_s_;      // 两次重规划之间的最短间隔，防止每拍重规划。
  int blocked_enter_cycles_;          // 连续多少拍阻挡才进入 BLOCKED，形成进入滞回。
  int clear_exit_cycles_;             // 连续多少拍通畅才退出 BLOCKED，形成退出滞回。
  int max_replan_attempts_;           // 最大重规划次数，超过则判恢复失败。
  double speed_filter_tau_;           // 一阶低通时间常数。大：更平滑；小：更跟手。
  double translational_acc_limit_;    // 恢复阶段平移加速度上限。
  double translational_decel_limit_;  // 恢复阶段平移减速度上限。
  double minimum_speed_xy_;           // 恢复阶段的最小平移速度，避免末段反复抖动。
  double high_cost_speed_threshold_;  // 超过该平均 cost 后开始下调恢复速度。
  double high_cost_speed_min_scale_;  // 高 cost 窄缝恢复时允许保留的最低速度比例。
  double goal_tolerance_;             // 恢复到终点的距离容差。
  double monitor_lookahead_distance_; // 恢复执行时前视检测长度。大：更早预判；小：更激进。
  bool enable_full_circle_fallback_;  // 侧后退都失败时，是否放开到全方向搜索。
};

}  // namespace ats_nav2_behaviors

#endif  // ATS_NAV2_PLUGINS__BEHAVIORS__BACK_UP_FREE_SPACE_HPP_
