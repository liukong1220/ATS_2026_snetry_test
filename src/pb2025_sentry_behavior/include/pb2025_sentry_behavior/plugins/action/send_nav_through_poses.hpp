#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV_THROUGH_POSES_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV_THROUGH_POSES_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class SendNavThroughPosesAction : public BT::SyncActionNode
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  SendNavThroughPosesAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // 读取行为树根黑板中的最新决策位姿。
  // 这里只把 decision_current_pose 当作“外部观测输入”，
  // 绝不能反向把动作节点内部缓存的位姿再写回黑板。
  // 否则在 loopback 手动拖车、RViz 设初始位姿、实车定位链路短时抖动时，
  // 旧位姿会覆盖掉真正的 odom / TF 更新，进而把视觉最近圆周点钉死在旧位置。
  std::optional<geometry_msgs::msg::PoseStamped> readObservedPoseFromBlackboard() const;
  // 当黑板位姿缺失或过期时，直接从 TF 查询当前车体位姿。
  // 这样视觉跟随和“是否仍然在终点附近”的判断可以在同一拍就拿到新车位，
  // 不必等到下一拍再由 server 的兜底逻辑刷新。
  std::optional<geometry_msgs::msg::PoseStamped> lookupObservedPoseFromTf(
    const std::string & target_frame) const;
  // 将观测到的当前位置转换到 path 所在坐标系。
  // loopback 手动拖车时，黑板里的位姿可能来自 odom，而视觉跟随 path 多数在 map；
  // 如果不在这里做 TF 对齐，就会把“同一个实体位置”误判成不同位置，或者反过来把
  // 明明已经换位的车仍误判成“还在终点附近”。
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToPathFrame(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & path_frame) const;
  // 判断当前活动目标是否仍然处于“已到点”状态。
  // 若机器人已经被重新放置到远离终点的位置，就应允许重新下发同一路径。
  bool isActiveGoalStillReached(const nav_msgs::msg::Path & path, double tolerance) const;
  void resultCallback(std::uint64_t request_id, const GoalHandle::WrappedResult & result);
  void feedbackCallback(
    std::uint64_t request_id, GoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  void goalResponseCallback(std::uint64_t request_id, const GoalHandle::SharedPtr & goal_handle);
  void cancelCurrentGoal();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SendNavThroughPosesAction");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  std::string action_name_;
  std::mutex mutex_;
  GoalHandle::SharedPtr current_goal_handle_;
  nav_msgs::msg::Path active_path_;
  geometry_msgs::msg::PoseStamped latest_pose_;
  bool has_current_pose_ = false;
  bool goal_pending_ = false;
  bool last_goal_succeeded_ = false;
  std::uint64_t goal_request_id_ = 0;
  // 用于判断“新路径是否还是同一个导航意图”的位置容差。
  double path_compare_tolerance_ = 0.1;
  // 用于判断“机器人是否真正已经到达该路径终点”的位置容差。
  // 该值应与 Nav2 SimpleGoalChecker 的 xy_goal_tolerance 保持一致，
  // 否则会出现“Nav2 已判成功，但行为层还认为没到点”的重复重发。
  double path_goal_reached_tolerance_ = 0.1;
  double action_server_wait_timeout_s_ = 2.0;
  double pose_timeout_s_ = 0.5;
  double active_goal_hold_tolerance_ = 0.45;
  double active_goal_min_resend_interval_s_ = 0.8;
  // 视觉实时跟随比普通巡逻/退防更强调“目标点要及时刷新”，
  // 因此单独拆出一组更积极的保持阈值与最小重发间隔。
  double vision_active_goal_hold_tolerance_ = 0.18;
  double vision_active_goal_min_resend_interval_s_ = 0.25;
  rclcpp::Time last_goal_sent_at_{0, 0, RCL_ROS_TIME};
  bool has_last_goal_sent_at_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV_THROUGH_POSES_HPP_
