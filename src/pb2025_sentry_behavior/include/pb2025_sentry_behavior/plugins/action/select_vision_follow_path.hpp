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
  BT::NodeStatus tick() override;
  std::string resolvePlanningFrame(
    const std::string & costmap_frame, const geometry_msgs::msg::PoseStamped & current_pose,
    const sp_msgs::msg::VisionTargetMsg & vision_target) const;
  std::optional<geometry_msgs::msg::Point> transformTargetPointToFrame(
    const sp_msgs::msg::VisionTargetMsg & vision_target, const std::string & target_frame) const;
  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToFrame(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame) const;
  bool shouldReuseCachedPath(
    const rclcpp::Time & now, const std::string & planning_frame,
    const geometry_msgs::msg::Point & selected_goal, double min_replan_interval_s,
    double min_goal_shift_m) const;
  void cachePath(
    const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & selected_goal, const nav_msgs::msg::Path & path,
    const std::string & planning_frame, const rclcpp::Time & now);
  void publishVisualization(
    const std::string & planning_frame, const geometry_msgs::msg::Point & current_position,
    const geometry_msgs::msg::Point & target_point, const geometry_msgs::msg::Point & selected_goal,
    double attack_radius, double preferred_angle, double arc_half_angle, int sample_count);
  void clearVisualization();
  void resetCachedPath();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("SelectVisionFollowPathAction");
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
  geometry_msgs::msg::Point last_target_point_;
  geometry_msgs::msg::Point last_selected_goal_;
  nav_msgs::msg::Path last_path_;
  std::optional<rclcpp::Time> last_plan_time_;
  std::string last_plan_frame_id_;
  bool has_cached_path_ = false;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_VISION_FOLLOW_PATH_HPP_
