// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__BSPLINE_PATH_OPTIMIZER_HPP_
#define TRAJECTORY_OPTIMIZER__BSPLINE_PATH_OPTIMIZER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace trajectory_optimizer
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

struct OptimizerParams
{
  double control_point_spacing = 0.30;
  double output_path_spacing = 0.05;
  double min_input_point_spacing = 0.02;
  double max_lateral_deviation = 0.18;
  int min_control_points = 5;
};

class BSplinePathOptimizer
{
public:
  explicit BSplinePathOptimizer(const OptimizerParams & params = OptimizerParams());

  void setParams(const OptimizerParams & params);
  const OptimizerParams & getParams() const;

  nav_msgs::msg::Path optimize(const nav_msgs::msg::Path & input_path) const;

private:
  std::vector<Point2D> extractPolyline(const nav_msgs::msg::Path & path) const;
  std::vector<Point2D> filterClosePoints(const std::vector<Point2D> & points) const;
  std::vector<Point2D> resamplePolyline(
    const std::vector<Point2D> & points, double spacing) const;
  std::vector<Point2D> buildSmoothedPolyline(const std::vector<Point2D> & points) const;
  std::vector<double> buildClampedUniformKnots(size_t control_points, size_t degree) const;
  size_t findKnotSpan(
    const std::vector<double> & knots, size_t degree, double u, size_t control_points) const;
  Point2D evaluateBSpline(
    const std::vector<Point2D> & control_points, const std::vector<double> & knots,
    size_t degree, double u) const;
  Point2D clampToCorridor(
    const Point2D & candidate, const std::vector<Point2D> & reference) const;
  std::pair<Point2D, double> closestPointOnPolyline(
    const Point2D & query, const std::vector<Point2D> & polyline) const;
  nav_msgs::msg::Path buildPathMessage(
    const std_msgs::msg::Header & header, const std::vector<Point2D> & points) const;

  static double distance(const Point2D & a, const Point2D & b);
  static double squaredDistance(const Point2D & a, const Point2D & b);
  static Point2D interpolate(const Point2D & a, const Point2D & b, double t);
  static double clampValue(double value, double min_value, double max_value);

  OptimizerParams params_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__BSPLINE_PATH_OPTIMIZER_HPP_
