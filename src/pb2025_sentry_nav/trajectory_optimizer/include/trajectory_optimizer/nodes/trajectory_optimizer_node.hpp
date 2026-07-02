// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_
#define TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_

#include <string>

#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sp_msgs/msg/trajectory_profile_msg.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "trajectory_optimizer/bspline/bspline_path_optimizer.hpp"
#include "trajectory_optimizer/esdf/esdf_provider.hpp"
#include "trajectory_optimizer/esdf/fake_costmap_esdf_provider.hpp"
#include "trajectory_optimizer/esdf/terrain_pointcloud_esdf_provider.hpp"
#include "trajectory_optimizer/esdf/rc_traversability_esdf_provider.hpp"

namespace trajectory_optimizer
{

class TrajectoryOptimizerNode : public rclcpp::Node
{
public:
  explicit TrajectoryOptimizerNode(const rclcpp::NodeOptions & options);

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void terrainPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void traversabilityGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void traversabilityHeightDiffCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void traversabilityOccupancyRatioCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void traversabilityGroundConfidenceCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void traversabilitySlopeCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publishEsdfDebugMarkers(const nav_msgs::msg::Path & path);
  void refreshEsdfProvider();
  void updateTraversabilityEsdf();

  BSplinePathOptimizer optimizer_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_height_diff_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_occupancy_ratio_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_ground_confidence_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_slope_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr smoothed_path_pub_;
  rclcpp::Publisher<sp_msgs::msg::TrajectoryProfileMsg>::SharedPtr profile_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr esdf_marker_pub_;
  std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
  std::shared_ptr<FakeCostmapEsdfProvider> fake_esdf_provider_;
  std::shared_ptr<TerrainPointCloudEsdfProvider> terrain_esdf_provider_;
  std::shared_ptr<RcTraversabilityEsdfProvider> traversability_esdf_provider_;
  EsdfProviderPtr active_esdf_provider_;
  OptimizerParams params_;

  std::string input_path_topic_;
  std::string output_path_topic_;
  std::string output_profile_topic_;
  std::string costmap_topic_;
  std::string esdf_debug_topic_{"trajectory_esdf_debug"};
  std::string esdf_source_{"costmap"};
  std::string terrain_pointcloud_topic_{"terrain_map_ext"};
  std::string traversability_grid_topic_{"traversability_grid"};
  std::string traversability_height_diff_topic_{"traversability_height_diff_grid"};
  std::string traversability_occupancy_ratio_topic_{"traversability_occupancy_ratio_grid"};
  std::string traversability_ground_confidence_topic_{"traversability_ground_confidence_grid"};
  std::string traversability_slope_topic_{"traversability_slope_grid"};
  double terrain_esdf_resolution_{0.05};
  double terrain_esdf_padding_{0.60};
  double terrain_esdf_inflation_radius_{0.08};
  double terrain_esdf_min_intensity_{0.0};
  bool rc_esdf_rolling_window_enabled_{true};
  double rc_esdf_query_window_size_x_{0.0};
  double rc_esdf_query_window_size_y_{0.0};
  double traversability_slope_max_degrees_{45.0};
  int traversability_obstacle_value_threshold_{50};
  int traversability_lethal_value_threshold_{90};
  bool traversability_unknown_is_obstacle_{false};
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_grid_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_height_diff_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_occupancy_ratio_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_ground_confidence_msg_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_slope_msg_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_NODE_HPP_
