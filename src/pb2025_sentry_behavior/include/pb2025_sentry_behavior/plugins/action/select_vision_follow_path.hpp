#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sp_msgs/msg/vision_target_msg.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb2025_sentry_behavior
{

class SelectVisionFollowPathAction : public BT::SyncActionNode
{
public:
  SelectVisionFollowPathAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  // 当 blackboard 中的 decision_current_pose 缺失或过期时，
  // 直接从 TF 查询当前车体位姿，避免继续拿旧位姿去计算“最近圆周点”。
  std::optional<geometry_msgs::msg::PoseStamped> lookupCurrentPoseFromTf(
    const std::string & target_frame) const;
  BT::NodeStatus tick() override;
  // 解析本次视觉跟随规划应使用的坐标系：
  // 优先 costmap frame，其次决策位姿 frame，最后退回视觉目标 map frame / map。
  std::string resolvePlanningFrame(
    const std::string & costmap_frame, const geometry_msgs::msg::PoseStamped & current_pose,
    const sp_msgs::msg::VisionTargetMsg & vision_target) const;
  // 将视觉目标中的地图点转换到规划坐标系，失败时返回空。
  std::optional<geometry_msgs::msg::Point> transformTargetPointToFrame(
    const sp_msgs::msg::VisionTargetMsg & vision_target, const std::string & target_frame) const;
  // 将机器人当前位姿转换到规划坐标系，便于基于“我在目标圆周上的哪一侧”来选最近跟随点。
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToFrame(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame) const;
  // 当定位结果出现“瞬时大跳变”时，说明这更像是手动重定位或定位链校正，
  // 而不是机器人靠自身运动自然走到了新位置。
  // 这类场景下若继续沿用上一拍的跟随侧和平滑缓存，视觉目标会先沿旧侧过渡几拍，
  // 看起来就像“最近圆周点没有立刻跟着变”。因此这里需要直接清空旧缓存。
  bool shouldResetCachedStateForPoseJump(
    const std::string & planning_frame, const geometry_msgs::msg::Point & current_position,
    double pose_jump_reset_distance_m) const;
  // 对本帧新计算出的圆周跟随点做角度限幅平滑。
  // 目标是“每拍都重新选点”，但不要因为目标轻抖或临近圆周切侧而瞬间大跳。
  geometry_msgs::msg::Point smoothSelectedGoal(
    const std::string & planning_frame, const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & raw_selected_goal, double max_goal_angle_step_rad) const;
  // 缓存上一拍视觉跟随结果。
  // update_plan_position_anchor=true 时，会同步刷新“上一次真正换目标时的机器人位姿”；
  // 若本拍只是因为死区而继续沿用旧目标，则保留旧参考位姿，让机器人位移能够累计，
  // 从而在后续真正触发重新选点。
  void cachePath(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & selected_goal, const nav_msgs::msg::Path & path,
    const std::string & planning_frame, const rclcpp::Time & now,
    const geometry_msgs::msg::Point & current_position, bool has_current_position,
    bool update_plan_position_anchor, const geometry_msgs::msg::Point & nearest_goal_point);
  void publishVisualization(
    const std::string & planning_frame, const geometry_msgs::msg::Point & current_position,
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & nearest_goal_point,
    const geometry_msgs::msg::Point & selected_goal, double attack_radius, double nearest_angle,
    double arc_half_angle, int sample_count);
  void clearVisualization();
  void resetCachedPath();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SelectVisionFollowPathAction");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  geometry_msgs::msg::Point last_target_point_;
  geometry_msgs::msg::Point last_nearest_goal_point_;
  geometry_msgs::msg::Point last_selected_goal_;
  std::optional<geometry_msgs::msg::Point> last_plan_position_;
  nav_msgs::msg::Path last_path_;
  std::optional<rclcpp::Time> last_plan_time_;
  std::string last_plan_frame_id_;
  bool has_cached_path_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_
