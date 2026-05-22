#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_CSV_WAYPOINTS_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_CSV_WAYPOINTS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb2025_sentry_behavior/decision_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{

class LoadCsvWaypointsAction : public BT::SyncActionNode
{
public:
  LoadCsvWaypointsAction(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;
  bool loadCsvIfNeeded(const std::string & filepath);
  nav_msgs::msg::Path buildFullPath() const;
  nav_msgs::msg::Path buildPatrolPath(int patrol_cursor) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_ = rclcpp::get_logger("LoadCsvWaypoints");
  std::vector<geometry_msgs::msg::Point> waypoints_;
  std::string loaded_filepath_;
  std::string last_logged_filepath_;
  bool last_logged_patrol_mode_ = false;
  int last_logged_patrol_cursor_ = -1;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_CSV_WAYPOINTS_HPP_
