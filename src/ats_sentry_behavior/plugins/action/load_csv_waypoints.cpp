#include "ats_sentry_behavior/plugins/action/load_csv_waypoints.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <utility>

namespace ats_sentry_behavior
{

namespace
{

std::string trim(const std::string & str)
{
  auto start = std::find_if_not(
    str.begin(), str.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
  auto end = std::find_if_not(str.rbegin(), str.rend(), [](unsigned char ch) {
               return std::isspace(ch) != 0;
             }).base();
  return (start < end) ? std::string(start, end) : std::string();
}

std::string resolveCsvPath(const std::string & filepath, const rclcpp::Logger & logger)
{
  if (std::filesystem::path(filepath).is_absolute()) {
    return filepath;
  }

  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("ats_sentry_behavior");
    return (std::filesystem::path(share_dir) / filepath).string();
  } catch (const std::exception & ex) {
    RCLCPP_WARN(logger, "Failed to resolve ats_sentry_behavior share directory: %s", ex.what());
    return filepath;
  }
}

}  // namespace

LoadCsvWaypointsAction::LoadCsvWaypointsAction(
  const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config),
  node_(decision::getNodeFromBlackboard(*this))
{
  logger_ = node_->get_logger();
}

BT::NodeStatus LoadCsvWaypointsAction::tick()
{
  std::string csv_file;
  if (!getInput("csv_file", csv_file) || csv_file.empty()) {
    RCLCPP_ERROR(logger_, "LoadCsvWaypoints did not receive a valid csv_file input");
    return BT::NodeStatus::FAILURE;
  }

  if (!loadCsvIfNeeded(csv_file)) {
    return BT::NodeStatus::FAILURE;
  }

  bool patrol_mode = false;
  getInput("patrol_mode", patrol_mode);

  nav_msgs::msg::Path path;
  int next_cursor = 0;
  int next_direction = 1;

  if (patrol_mode) {
    int patrol_cursor = 0;
    int patrol_direction = 1;
    getInput("patrol_cursor", patrol_cursor);
    getInput("patrol_direction", patrol_direction);

    if (patrol_cursor < 0 || patrol_cursor >= static_cast<int>(waypoints_.size())) {
      patrol_cursor = 0;
    }
    if (patrol_direction == 0) {
      patrol_direction = 1;
    }

    path = buildPatrolPath(patrol_cursor);
    if (waypoints_.size() > 1) {
      const auto next_state =
        decision::computeNextPatrolState(patrol_cursor, patrol_direction, waypoints_.size());
      next_cursor = next_state.first;
      next_direction = next_state.second;
    }

    if (
      loaded_filepath_ != last_logged_filepath_ || !last_logged_patrol_mode_ ||
      patrol_cursor != last_logged_patrol_cursor_)
    {
      const auto & point = waypoints_.at(static_cast<std::size_t>(patrol_cursor));
      RCLCPP_INFO(
        logger_,
        "[%s] patrol waypoint selected: file=%s cursor=%d point=(%.2f, %.2f, %.2f) next_cursor=%d next_direction=%d",
        name().c_str(), loaded_filepath_.c_str(), patrol_cursor,
        point.x, point.y, point.z, next_cursor, next_direction);
      last_logged_filepath_ = loaded_filepath_;
      last_logged_patrol_mode_ = true;
      last_logged_patrol_cursor_ = patrol_cursor;
    }
  } else {
    path = buildFullPath();
    if (loaded_filepath_ != last_logged_filepath_ || last_logged_patrol_mode_) {
      const auto & first_point = waypoints_.front();
      const auto & last_point = waypoints_.back();
      RCLCPP_INFO(
        logger_,
        "[%s] mapping route selected: file=%s count=%zu first=(%.2f, %.2f, %.2f) last=(%.2f, %.2f, %.2f)",
        name().c_str(), loaded_filepath_.c_str(), waypoints_.size(),
        first_point.x, first_point.y, first_point.z,
        last_point.x, last_point.y, last_point.z);
      last_logged_filepath_ = loaded_filepath_;
      last_logged_patrol_mode_ = false;
      last_logged_patrol_cursor_ = -1;
    }
  }

  if (path.poses.empty()) {
    RCLCPP_ERROR(logger_, "LoadCsvWaypoints built an empty path from %s", csv_file.c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("path", path);
  setOutput("next_cursor", next_cursor);
  setOutput("next_direction", next_direction);
  return BT::NodeStatus::SUCCESS;
}

bool LoadCsvWaypointsAction::loadCsvIfNeeded(const std::string & filepath)
{
  if (filepath == loaded_filepath_ && !waypoints_.empty()) {
    return true;
  }

  const auto resolved_path = resolveCsvPath(filepath, logger_);
  std::ifstream file(resolved_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger_, "Cannot open CSV waypoint file: %s", resolved_path.c_str());
    return false;
  }

  std::vector<geometry_msgs::msg::Point> parsed_points;
  std::string line;
  int line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    const auto trimmed = trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    std::istringstream line_stream(trimmed);
    std::string token;
    std::vector<double> values;
    while (std::getline(line_stream, token, ',')) {
      try {
        values.push_back(std::stod(trim(token)));
      } catch (const std::exception &) {
        RCLCPP_WARN(
          logger_, "Skip malformed value in %s line %d: '%s'", resolved_path.c_str(), line_number,
          token.c_str());
        values.clear();
        break;
      }
    }

    if (values.size() == 3) {
      geometry_msgs::msg::Point point;
      point.x = values[0];
      point.y = values[1];
      point.z = values[2];
      parsed_points.push_back(point);
      continue;
    }

    if (!values.empty()) {
      RCLCPP_WARN(
        logger_, "Skip line %d in %s: expected x,y,z, got %zu values", line_number,
        resolved_path.c_str(), values.size());
    }
  }

  if (parsed_points.empty()) {
    RCLCPP_ERROR(logger_, "CSV waypoint file contains no valid point: %s", resolved_path.c_str());
    return false;
  }

  waypoints_ = std::move(parsed_points);
  loaded_filepath_ = filepath;
  RCLCPP_INFO(
    logger_, "Loaded %zu CSV waypoints from %s", waypoints_.size(), resolved_path.c_str());
  return true;
}

nav_msgs::msg::Path LoadCsvWaypointsAction::buildFullPath() const
{
  std::vector<std::size_t> indices;
  indices.reserve(waypoints_.size());
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    indices.push_back(i);
  }
  return decision::buildPathFromIndices(waypoints_, indices);
}

nav_msgs::msg::Path LoadCsvWaypointsAction::buildPatrolPath(int patrol_cursor) const
{
  if (waypoints_.empty()) {
    return nav_msgs::msg::Path{};
  }

  if (patrol_cursor < 0 || patrol_cursor >= static_cast<int>(waypoints_.size())) {
    patrol_cursor = 0;
  }

  return decision::buildPathFromIndices(waypoints_, {static_cast<std::size_t>(patrol_cursor)});
}

BT::PortsList LoadCsvWaypointsAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("csv_file", "{decision_rmuc_csv_file}", "CSV waypoint file"),
    BT::InputPort<bool>(
      "patrol_mode", false, "If true, output one patrol target from the CSV cursor"),
    BT::InputPort<int>("patrol_cursor", "{decision_patrol_cursor}", "Current patrol cursor"),
    BT::InputPort<int>(
      "patrol_direction", "{decision_patrol_direction}", "Current patrol direction"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "{decision_path}", "Path loaded from CSV"),
    BT::OutputPort<int>("next_cursor", "{decision_next_patrol_cursor}", "Next patrol cursor"),
    BT::OutputPort<int>(
      "next_direction", "{decision_next_patrol_direction}", "Next patrol direction")};
}

}  // namespace ats_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ats_sentry_behavior::LoadCsvWaypointsAction>("LoadCsvWaypoints");
}
