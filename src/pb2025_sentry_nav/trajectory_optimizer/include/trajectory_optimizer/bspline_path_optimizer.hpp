// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__BSPLINE_PATH_OPTIMIZER_HPP_
#define TRAJECTORY_OPTIMIZER__BSPLINE_PATH_OPTIMIZER_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"
#include "trajectory_optimizer/esdf_provider.hpp"

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
  double curvature_limit = 1.2;
  double curvature_weight = 20.0;
  int curvature_refinement_iterations = 3;
  double curvature_refinement_gain = 0.02;
  double global_speed_limit = 1.6;
  double lateral_accel_limit = 1.0;
  double longitudinal_accel_limit = 0.8;
  double velocity_smoothing_gain = 0.2;
  double derivative_step = 0.02;
  unsigned char obstacle_safe_cost = 64;
  double obstacle_weight = 25.0;
  int obstacle_refinement_iterations = 3;
  double obstacle_refinement_gain = 0.03;
  bool use_esdf_obstacle_cost = false;
  double obstacle_safe_distance = 0.30;
};

struct TrajectorySample2D
{
  double s = 0.0;
  double t = 0.0;
  Point2D point;
  Point2D first_derivative;
  Point2D second_derivative;
  double curvature = 0.0;
  double speed_limit = 0.0;
  double speed = 0.0;
  double acceleration = 0.0;
};

struct TrajectoryProfile2D
{
  std::vector<TrajectorySample2D> samples;
  double total_length = 0.0;
  double total_time = 0.0;
  double curvature_penalty = 0.0;
  double velocity_smoothness_cost = 0.0;
  double obstacle_cost = 0.0;
  double total_cost = 0.0;
  double max_abs_curvature = 0.0;
};

struct OptimizationResult
{
  nav_msgs::msg::Path path;
  TrajectoryProfile2D profile;
};

class CubicBSpline2D
{
public:
  CubicBSpline2D() = default;
  CubicBSpline2D(
    const std::vector<Point2D> & control_points,
    double derivative_step);

  bool valid() const;
  double totalLength() const;

  Point2D getPoint(double s) const;
  Point2D getFirstDerivative(double s) const;
  Point2D getSecondDerivative(double s) const;
  double getCurvature(double s) const;

private:
  void rebuildArcLengthTable();
  double clampArcLength(double s) const;
  double parameterFromArcLength(double s) const;
  Point2D evaluateByParameter(double u) const;
  size_t findKnotSpan(double u) const;
  std::vector<double> buildClampedUniformKnots(size_t control_points, size_t degree) const;
  static Point2D interpolate(const Point2D & a, const Point2D & b, double t);
  static double distance(const Point2D & a, const Point2D & b);
  static double clampValue(double value, double min_value, double max_value);

  std::vector<Point2D> control_points_;
  std::vector<double> knots_;
  std::vector<double> sampled_parameters_;
  std::vector<double> sampled_arc_lengths_;
  double total_length_ = 0.0;
  double derivative_step_ = 0.02;
  size_t degree_ = 3;
};

class BSplinePathOptimizer
{
public:
  explicit BSplinePathOptimizer(const OptimizerParams & params = OptimizerParams());

  void setParams(const OptimizerParams & params);
  const OptimizerParams & getParams() const;
  void setObstacleCostmap(const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap);
  void clearObstacleCostmap();
  void setEsdfProvider(const EsdfProviderPtr & provider);
  void clearEsdfProvider();

  OptimizationResult optimizeDetailed(const nav_msgs::msg::Path & input_path) const;
  nav_msgs::msg::Path optimize(const nav_msgs::msg::Path & input_path) const;
  TrajectoryProfile2D evaluateProfile(const nav_msgs::msg::Path & path) const;

private:
  std::vector<Point2D> extractPolyline(const nav_msgs::msg::Path & path) const;
  std::vector<Point2D> filterClosePoints(const std::vector<Point2D> & points) const;
  std::vector<Point2D> resamplePolyline(
    const std::vector<Point2D> & points, double spacing) const;
  CubicBSpline2D buildSpline(const std::vector<Point2D> & points) const;
  std::vector<Point2D> sampleSplineDense(const CubicBSpline2D & spline) const;
  std::vector<Point2D> refinePathForCurvature(
    const std::vector<Point2D> & dense_points,
    const std::vector<Point2D> & reference) const;
  std::vector<Point2D> refinePathUnified(
    const std::vector<Point2D> & dense_points,
    const std::vector<Point2D> & reference) const;
  std::vector<Point2D> buildSmoothedPolyline(const std::vector<Point2D> & points) const;
  TrajectoryProfile2D buildTrajectoryProfile(const std::vector<Point2D> & points) const;
  void applyCurvatureSpeedLimits(TrajectoryProfile2D & profile) const;
  void applyAccelerationLimits(TrajectoryProfile2D & profile) const;
  void smoothVelocityProfile(TrajectoryProfile2D & profile) const;
  static Point2D discreteFirstDerivative(
    const std::vector<Point2D> & points, size_t index, double ds);
  static Point2D discreteSecondDerivative(
    const std::vector<Point2D> & points, size_t index, double ds);
  static double computeCurvature(
    const Point2D & first_derivative,
    const Point2D & second_derivative);
  bool sampleObstacleCost(
    const Point2D & point,
    unsigned char & cost) const;
  bool sampleEsdfDistance(
    const Point2D & point,
    double & distance) const;
  Point2D estimateObstacleGradient(
    const Point2D & point) const;
  Point2D estimateEsdfGradient(
    const Point2D & point) const;
  double computeObstaclePenalty(
    unsigned char cost) const;
  double computeObstaclePenaltyFromDistance(
    double distance) const;
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
  std::shared_ptr<nav2_costmap_2d::Costmap2D> obstacle_costmap_;
  EsdfProviderPtr esdf_provider_;
};

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__BSPLINE_PATH_OPTIMIZER_HPP_
