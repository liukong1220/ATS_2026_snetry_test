// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
#include "minco_planner/debug/planner_debug_visualizer.hpp"
#include "minco_planner/trajectory/yaw_spline_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace minco_planner
{

class MincoPlannerNode : public rclcpp::Node
{
public:
  explicit MincoPlannerNode(const rclcpp::NodeOptions & options);

private:
  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool lookupStartPose(geometry_msgs::msg::PoseStamped & start) const;
  nav_msgs::msg::Path toPath(const ReferenceTrajectory & trajectory) const;
  void declareAndLoadParams();

  std::string grid_topic_ = "traversability_grid";
  std::string goal_topic_ = "goal_pose";
  std::string raw_path_topic_ = "minco/raw_path";
  std::string reference_path_topic_ = "minco/reference_path";
  std::string debug_marker_topic_ = "minco/debug_markers";
  std::string global_frame_ = "map";
  std::string robot_frame_ = "base_link";

  GridAstar astar_;
  MincoTrajectoryOptimizer optimizer_;
  YawSplinePlanner yaw_planner_;
  FootprintSafetyChecker safety_checker_;
  LocalCollisionRepair collision_repair_;
  PlannerDebugVisualizer visualizer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_
