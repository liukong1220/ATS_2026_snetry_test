 

#ifndef STANDARD_ROBOT_PP_ROS2__2025_ROBOT_DECISION_HPP_
#define STANDARD_ROBOT_PP_ROS2__2025_ROBOT_DECISION_HPP_

#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "pb_rm_interfaces/msg/game_status.hpp"
#include "pb_rm_interfaces/msg/rfid_status.hpp"
#include "pb_rm_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"

namespace standard_robot_pp_ros2
{

class RobotDecisionNode : public rclcpp::Node
{
public:
  explicit RobotDecisionNode(const rclcpp::NodeOptions & options);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  struct DecisionParams
  {
    struct TimeThresholds
    {
      int abundant;
      int normal;
      int tense;
      int critical;
    } time;

    struct HpThresholds
    {
      struct TimeStages
      {
        int high;
        int medium;
        int low;
      };

      TimeStages abundant;
      TimeStages normal;
      TimeStages tense;
      TimeStages critical;
    } hp;

    struct DecisionConfig
    {
      double path_tolerance;
      std::string nav2_action_server;
      int decision_period_ms;
      double goal_republish_interval_s;
      double goal_position_tolerance;
      double waypoint_stop_duration_s;
    } config;
  };

  void gameStatusCallback(const pb_rm_interfaces::msg::GameStatus::SharedPtr msg);
  void rfidStatusCallback(const pb_rm_interfaces::msg::RfidStatus::SharedPtr msg);
  void robotStatusCallback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

  void executeDecisionLogic();
  void sendNavigationGoal(float x, float y, float z = 0.0f, float yaw = 0.0f);
  int getGameTimeState();
  void robotDecision();
  void handleNormalHpBehavior(double tolerance);
  void handleLowHpBehavior(double tolerance);
  void handleCriticalHpBehavior(double tolerance);
  size_t findNearestPoint(
    const geometry_msgs::msg::Point & reference_point,
    const std::vector<size_t> & candidate_indices);
  bool checkDistance(const geometry_msgs::msg::Point & target_point, double tolerance);
  void logDecisionState(int hp, size_t index);
  void triggerStopSequence();
  void publishStopFlag(bool stop);
  void updateStopState();

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
  GoalHandleNavigate::SharedPtr current_goal_handle_;

  std::vector<geometry_msgs::msg::Point> rmul_goal_point_;
  DecisionParams params_;

  size_t supply_safe_point_index_ = 0;
  std::vector<size_t> patrol_indices_;
  std::vector<size_t> low_hp_candidate_indices_;
  size_t critical_time_target_index_ = 0;
  int patrol_direction_ = 1;
  size_t patrol_cursor_ = 0;
  size_t low_hp_target_index_ = std::numeric_limits<size_t>::max();

  std::mutex nav_mutex_;
  pb_rm_interfaces::msg::GameStatus game_status_msg_;
  pb_rm_interfaces::msg::RfidStatus rfid_status_msg_;
  pb_rm_interfaces::msg::RobotStatus robot_status_msg_;
  geometry_msgs::msg::PoseStamped current_pose_;

  rclcpp::TimerBase::SharedPtr stop_reset_timer_;
  NavigateToPose::Feedback::SharedPtr current_feedback_;
  rclcpp_action::ResultCode last_nav_result_;
  bool robot_move_;
  bool has_feedback_ = false;
  bool has_last_goal_ = false;
  bool goal_request_pending_ = false;
  geometry_msgs::msg::Point last_goal_point_;
  double last_goal_yaw_ = 0.0;
  rclcpp::Time last_goal_send_time_;
  bool stop_flag_state_ = false;
  bool stop_hold_active_ = false;
  rclcpp::Time stop_release_time_;
  int last_game_progress_ = -1;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<pb_rm_interfaces::msg::GameStatus>::SharedPtr game_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RfidStatus>::SharedPtr rfid_status_sub_;
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_flag_pub_;
};

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__2025_ROBOT_DECISION_HPP_
