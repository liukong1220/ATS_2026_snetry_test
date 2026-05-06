// Copyright 2026

#include "trajectory_optimizer/bspline_path_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace trajectory_optimizer
{

namespace
{

constexpr size_t kSplineDegree = 3;
constexpr double kEpsilon = 1e-9;

}  // namespace

BSplinePathOptimizer::BSplinePathOptimizer(const OptimizerParams & params)
: params_(params)
{
}

void BSplinePathOptimizer::setParams(const OptimizerParams & params)
{
  params_ = params;
}

const OptimizerParams & BSplinePathOptimizer::getParams() const
{
  return params_;
}

nav_msgs::msg::Path BSplinePathOptimizer::optimize(const nav_msgs::msg::Path & input_path) const
{
  const auto raw_points = extractPolyline(input_path);
  const auto filtered_points = filterClosePoints(raw_points);
  if (filtered_points.size() < static_cast<size_t>(std::max(2, params_.min_control_points))) {
    return buildPathMessage(input_path.header, filtered_points);
  }

  return buildPathMessage(input_path.header, buildSmoothedPolyline(filtered_points));
}

std::vector<Point2D> BSplinePathOptimizer::extractPolyline(const nav_msgs::msg::Path & path) const
{
  std::vector<Point2D> points;
  points.reserve(path.poses.size());
  for (const auto & pose_stamped : path.poses) {
    points.push_back(Point2D {
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y});
  }
  return points;
}

std::vector<Point2D> BSplinePathOptimizer::filterClosePoints(
  const std::vector<Point2D> & points) const
{
  if (points.empty()) {
    return {};
  }

  std::vector<Point2D> filtered;
  filtered.reserve(points.size());
  filtered.push_back(points.front());

  for (size_t i = 1; i < points.size(); ++i) {
    if (distance(points[i], filtered.back()) >= params_.min_input_point_spacing) {
      filtered.push_back(points[i]);
    }
  }

  if (filtered.size() == 1 && points.size() > 1) {
    filtered.push_back(points.back());
  } else if (filtered.back().x != points.back().x || filtered.back().y != points.back().y) {
    filtered.push_back(points.back());
  }

  return filtered;
}

std::vector<Point2D> BSplinePathOptimizer::resamplePolyline(
  const std::vector<Point2D> & points, double spacing) const
{
  if (points.size() < 2 || spacing <= kEpsilon) {
    return points;
  }

  std::vector<double> cumulative_lengths(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    cumulative_lengths[i] = cumulative_lengths[i - 1] + distance(points[i - 1], points[i]);
  }

  const double total_length = cumulative_lengths.back();
  if (total_length <= spacing) {
    return {points.front(), points.back()};
  }

  std::vector<Point2D> resampled;
  const size_t sample_count = static_cast<size_t>(std::floor(total_length / spacing)) + 1;
  resampled.reserve(sample_count + 1);

  size_t segment_index = 1;
  for (size_t sample = 0; sample <= sample_count; ++sample) {
    const double target_length = std::min(total_length, sample * spacing);
    while (
      segment_index < cumulative_lengths.size() &&
      cumulative_lengths[segment_index] < target_length)
    {
      ++segment_index;
    }

    if (segment_index >= cumulative_lengths.size()) {
      break;
    }

    const double start_length = cumulative_lengths[segment_index - 1];
    const double end_length = cumulative_lengths[segment_index];
    const double segment_length = std::max(kEpsilon, end_length - start_length);
    const double ratio = clampValue(
      (target_length - start_length) / segment_length, 0.0, 1.0);
    resampled.push_back(interpolate(points[segment_index - 1], points[segment_index], ratio));
  }

  if (resampled.empty() || distance(resampled.back(), points.back()) > params_.min_input_point_spacing) {
    resampled.push_back(points.back());
  }

  return resampled;
}

std::vector<Point2D> BSplinePathOptimizer::buildSmoothedPolyline(
  const std::vector<Point2D> & points) const
{
  const auto control_points = resamplePolyline(points, params_.control_point_spacing);
  if (control_points.size() < std::max(kSplineDegree + 1, static_cast<size_t>(params_.min_control_points))) {
    return resamplePolyline(points, params_.output_path_spacing);
  }

  const auto knots = buildClampedUniformKnots(control_points.size(), kSplineDegree);

  double total_length = 0.0;
  for (size_t i = 1; i < points.size(); ++i) {
    total_length += distance(points[i - 1], points[i]);
  }
  const size_t sample_count =
    std::max<size_t>(2, static_cast<size_t>(std::ceil(total_length / params_.output_path_spacing)) + 1);

  std::vector<Point2D> smoothed;
  smoothed.reserve(sample_count);
  for (size_t i = 0; i < sample_count; ++i) {
    const double u = (sample_count == 1) ? 0.0 : static_cast<double>(i) / (sample_count - 1);
    const auto candidate = evaluateBSpline(control_points, knots, kSplineDegree, u);
    smoothed.push_back(clampToCorridor(candidate, points));
  }

  smoothed.front() = points.front();
  smoothed.back() = points.back();
  return resamplePolyline(smoothed, params_.output_path_spacing);
}

std::vector<double> BSplinePathOptimizer::buildClampedUniformKnots(
  size_t control_points, size_t degree) const
{
  const size_t knot_count = control_points + degree + 1;
  std::vector<double> knots(knot_count, 0.0);
  const size_t interior_count = control_points - degree - 1;

  for (size_t i = 0; i <= degree; ++i) {
    knots[knot_count - 1 - i] = 1.0;
  }

  for (size_t i = 1; i <= interior_count; ++i) {
    knots[degree + i] = static_cast<double>(i) / (interior_count + 1);
  }

  return knots;
}

size_t BSplinePathOptimizer::findKnotSpan(
  const std::vector<double> & knots, size_t degree, double u, size_t control_points) const
{
  if (u >= 1.0) {
    return control_points - 1;
  }

  size_t low = degree;
  size_t high = control_points;
  size_t mid = (low + high) / 2;
  while (u < knots[mid] || u >= knots[mid + 1]) {
    if (u < knots[mid]) {
      high = mid;
    } else {
      low = mid;
    }
    mid = (low + high) / 2;
  }
  return mid;
}

Point2D BSplinePathOptimizer::evaluateBSpline(
  const std::vector<Point2D> & control_points, const std::vector<double> & knots,
  size_t degree, double u) const
{
  const size_t span = findKnotSpan(knots, degree, u, control_points.size());
  std::vector<Point2D> work_points(degree + 1);
  for (size_t j = 0; j <= degree; ++j) {
    work_points[j] = control_points[span - degree + j];
  }

  for (size_t r = 1; r <= degree; ++r) {
    for (int j = static_cast<int>(degree); j >= static_cast<int>(r); --j) {
      const size_t knot_index = span - degree + static_cast<size_t>(j);
      const double denominator = knots[knot_index + degree + 1 - r] - knots[knot_index];
      const double alpha = (std::abs(denominator) <= kEpsilon) ? 0.0 :
        (u - knots[knot_index]) / denominator;
      work_points[static_cast<size_t>(j)] = interpolate(
        work_points[static_cast<size_t>(j - 1)],
        work_points[static_cast<size_t>(j)],
        clampValue(alpha, 0.0, 1.0));
    }
  }

  return work_points[degree];
}

Point2D BSplinePathOptimizer::clampToCorridor(
  const Point2D & candidate, const std::vector<Point2D> & reference) const
{
  if (params_.max_lateral_deviation <= 0.0 || reference.size() < 2) {
    return candidate;
  }

  const auto closest_result = closestPointOnPolyline(candidate, reference);
  if (closest_result.second <= params_.max_lateral_deviation) {
    return candidate;
  }

  const double dx = candidate.x - closest_result.first.x;
  const double dy = candidate.y - closest_result.first.y;
  const double norm = std::max(kEpsilon, std::sqrt(dx * dx + dy * dy));
  const double scale = params_.max_lateral_deviation / norm;
  return Point2D {
    closest_result.first.x + dx * scale,
    closest_result.first.y + dy * scale};
}

std::pair<Point2D, double> BSplinePathOptimizer::closestPointOnPolyline(
  const Point2D & query, const std::vector<Point2D> & polyline) const
{
  Point2D best_point = polyline.front();
  double best_distance = std::numeric_limits<double>::max();

  for (size_t i = 1; i < polyline.size(); ++i) {
    const Point2D & a = polyline[i - 1];
    const Point2D & b = polyline[i];
    const double seg_dx = b.x - a.x;
    const double seg_dy = b.y - a.y;
    const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

    double t = 0.0;
    if (seg_len_sq > kEpsilon) {
      t = ((query.x - a.x) * seg_dx + (query.y - a.y) * seg_dy) / seg_len_sq;
      t = clampValue(t, 0.0, 1.0);
    }

    const Point2D projection = interpolate(a, b, t);
    const double dist = distance(query, projection);
    if (dist < best_distance) {
      best_distance = dist;
      best_point = projection;
    }
  }

  return {best_point, best_distance};
}

nav_msgs::msg::Path BSplinePathOptimizer::buildPathMessage(
  const std_msgs::msg::Header & header, const std::vector<Point2D> & points) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(points.size());

  if (points.empty()) {
    return path;
  }

  double last_yaw = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i + 1 < points.size()) {
      last_yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
    }

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose.position.x = points[i].x;
    pose_stamped.pose.position.y = points[i].y;
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation.z = std::sin(last_yaw * 0.5);
    pose_stamped.pose.orientation.w = std::cos(last_yaw * 0.5);
    path.poses.push_back(pose_stamped);
  }

  return path;
}

double BSplinePathOptimizer::distance(const Point2D & a, const Point2D & b)
{
  return std::sqrt(squaredDistance(a, b));
}

double BSplinePathOptimizer::squaredDistance(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

Point2D BSplinePathOptimizer::interpolate(const Point2D & a, const Point2D & b, double t)
{
  return Point2D {
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t};
}

double BSplinePathOptimizer::clampValue(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace trajectory_optimizer
