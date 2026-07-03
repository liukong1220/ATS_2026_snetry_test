// Copyright 2026

#include "minco_planner/debug/planner_debug_visualizer.hpp"

#include <cmath>

#include "std_msgs/msg/color_rgba.hpp"

namespace minco_planner
{

namespace
{

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA out;
  out.r = r;
  out.g = g;
  out.b = b;
  out.a = a;
  return out;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

}  // namespace

visualization_msgs::msg::MarkerArray PlannerDebugVisualizer::buildMarkers(
  const nav_msgs::msg::Path & raw_path,
  const ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & safety) const
{
  visualization_msgs::msg::MarkerArray markers;
  const auto header = trajectory.header.frame_id.empty() ? raw_path.header : trajectory.header;

  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.ns = "minco_planner";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker raw;
  raw.header = raw_path.header;
  raw.ns = "minco_planner_raw";
  raw.id = 1;
  raw.type = visualization_msgs::msg::Marker::LINE_STRIP;
  raw.action = visualization_msgs::msg::Marker::ADD;
  raw.scale.x = 0.035;
  raw.color = color(0.1F, 0.55F, 1.0F, 0.9F);
  for (const auto & pose : raw_path.poses) {
    raw.points.push_back(pose.pose.position);
  }
  markers.markers.push_back(raw);

  visualization_msgs::msg::Marker reference;
  reference.header = trajectory.header;
  reference.ns = "minco_planner_reference";
  reference.id = 2;
  reference.type = visualization_msgs::msg::Marker::LINE_STRIP;
  reference.action = visualization_msgs::msg::Marker::ADD;
  reference.scale.x = 0.045;
  reference.color = color(0.15F, 0.9F, 0.35F, 0.95F);
  for (const auto & point : trajectory.points) {
    reference.points.push_back(makePoint(point.x, point.y, 0.05));
  }
  markers.markers.push_back(reference);

  visualization_msgs::msg::Marker yaw;
  yaw.header = trajectory.header;
  yaw.ns = "minco_planner_yaw";
  yaw.id = 3;
  yaw.type = visualization_msgs::msg::Marker::LINE_LIST;
  yaw.action = visualization_msgs::msg::Marker::ADD;
  yaw.scale.x = 0.018;
  yaw.color = color(1.0F, 0.82F, 0.15F, 0.9F);
  for (std::size_t i = 0; i < trajectory.points.size(); i += 5) {
    const auto & point = trajectory.points[i];
    yaw.points.push_back(makePoint(point.x, point.y, 0.08));
    yaw.points.push_back(
      makePoint(point.x + 0.22 * std::cos(point.yaw), point.y + 0.22 * std::sin(point.yaw), 0.08));
  }
  markers.markers.push_back(yaw);

  visualization_msgs::msg::Marker collisions;
  collisions.header = trajectory.header;
  collisions.ns = "minco_planner_collisions";
  collisions.id = 4;
  collisions.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  collisions.action = visualization_msgs::msg::Marker::ADD;
  collisions.scale.x = 0.12;
  collisions.scale.y = 0.12;
  collisions.scale.z = 0.12;
  collisions.color = color(1.0F, 0.1F, 0.1F, 0.95F);
  for (const auto & collision : safety.collisions) {
    collisions.points.push_back(makePoint(collision.x, collision.y, 0.14));
  }
  markers.markers.push_back(collisions);

  return markers;
}

}  // namespace minco_planner
