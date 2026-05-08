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
constexpr size_t kArcLengthSamples = 400;

}  // namespace

CubicBSpline2D::CubicBSpline2D(
  const std::vector<Point2D> & control_points,
  double derivative_step)
: control_points_(control_points),
  derivative_step_(std::max(1e-3, derivative_step))
{
  if (control_points_.size() >= degree_ + 1) {
    knots_ = buildClampedUniformKnots(control_points_.size(), degree_);
    rebuildArcLengthTable();
  }
}

bool CubicBSpline2D::valid() const
{
  return control_points_.size() >= degree_ + 1 && !sampled_arc_lengths_.empty();
}

double CubicBSpline2D::totalLength() const
{
  return total_length_;
}

Point2D CubicBSpline2D::getPoint(double s) const
{
  if (!valid()) {
    return {};
  }
  return evaluateByParameter(parameterFromArcLength(s));
}

Point2D CubicBSpline2D::getFirstDerivative(double s) const
{
  if (!valid()) {
    return {};
  }

  const double clamped_s = clampArcLength(s);
  const double left_s = std::max(0.0, clamped_s - derivative_step_);
  const double right_s = std::min(total_length_, clamped_s + derivative_step_);
  if (right_s - left_s <= kEpsilon) {
    return {};
  }

  const auto left = getPoint(left_s);
  const auto right = getPoint(right_s);
  const double inv = 1.0 / (right_s - left_s);
  return Point2D {(right.x - left.x) * inv, (right.y - left.y) * inv};
}

Point2D CubicBSpline2D::getSecondDerivative(double s) const
{
  if (!valid()) {
    return {};
  }

  const double clamped_s = clampArcLength(s);
  const double left_s = std::max(0.0, clamped_s - derivative_step_);
  const double right_s = std::min(total_length_, clamped_s + derivative_step_);
  if (right_s - left_s <= kEpsilon) {
    return {};
  }

  const auto center = getPoint(clamped_s);
  const auto left = getPoint(left_s);
  const auto right = getPoint(right_s);
  const double ds_left = clamped_s - left_s;
  const double ds_right = right_s - clamped_s;
  const double ds = std::max(kEpsilon, 0.5 * (ds_left + ds_right));
  const double inv = 1.0 / (ds * ds);
  return Point2D {
    (right.x - 2.0 * center.x + left.x) * inv,
    (right.y - 2.0 * center.y + left.y) * inv};
}

double CubicBSpline2D::getCurvature(double s) const
{
  const auto first = getFirstDerivative(s);
  const auto second = getSecondDerivative(s);
  const double numerator = first.x * second.y - first.y * second.x;
  const double denom_sq = first.x * first.x + first.y * first.y;
  if (denom_sq <= kEpsilon) {
    return 0.0;
  }
  return numerator / std::pow(denom_sq, 1.5);
}

void CubicBSpline2D::rebuildArcLengthTable()
{
  sampled_parameters_.clear();
  sampled_arc_lengths_.clear();
  sampled_parameters_.reserve(kArcLengthSamples + 1);
  sampled_arc_lengths_.reserve(kArcLengthSamples + 1);

  total_length_ = 0.0;
  Point2D previous = evaluateByParameter(0.0);
  sampled_parameters_.push_back(0.0);
  sampled_arc_lengths_.push_back(0.0);

  for (size_t i = 1; i <= kArcLengthSamples; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(kArcLengthSamples);
    Point2D current = evaluateByParameter(u);
    total_length_ += distance(previous, current);
    sampled_parameters_.push_back(u);
    sampled_arc_lengths_.push_back(total_length_);
    previous = current;
  }
}

double CubicBSpline2D::clampArcLength(double s) const
{
  return clampValue(s, 0.0, total_length_);
}

double CubicBSpline2D::parameterFromArcLength(double s) const
{
  const double clamped_s = clampArcLength(s);
  auto upper = std::lower_bound(
    sampled_arc_lengths_.begin(), sampled_arc_lengths_.end(), clamped_s);
  if (upper == sampled_arc_lengths_.begin()) {
    return sampled_parameters_.front();
  }
  if (upper == sampled_arc_lengths_.end()) {
    return sampled_parameters_.back();
  }

  const size_t idx = static_cast<size_t>(std::distance(sampled_arc_lengths_.begin(), upper));
  const double left_s = sampled_arc_lengths_[idx - 1];
  const double right_s = sampled_arc_lengths_[idx];
  const double ratio = (right_s - left_s <= kEpsilon) ? 0.0 :
    (clamped_s - left_s) / (right_s - left_s);
  return sampled_parameters_[idx - 1] +
    (sampled_parameters_[idx] - sampled_parameters_[idx - 1]) * clampValue(ratio, 0.0, 1.0);
}

Point2D CubicBSpline2D::evaluateByParameter(double u) const
{
  const size_t span = findKnotSpan(u);
  std::vector<Point2D> work_points(degree_ + 1);
  for (size_t j = 0; j <= degree_; ++j) {
    work_points[j] = control_points_[span - degree_ + j];
  }

  for (size_t r = 1; r <= degree_; ++r) {
    for (int j = static_cast<int>(degree_); j >= static_cast<int>(r); --j) {
      const size_t knot_index = span - degree_ + static_cast<size_t>(j);
      const double denominator = knots_[knot_index + degree_ + 1 - r] - knots_[knot_index];
      const double alpha = (std::abs(denominator) <= kEpsilon) ? 0.0 :
        (u - knots_[knot_index]) / denominator;
      work_points[static_cast<size_t>(j)] = interpolate(
        work_points[static_cast<size_t>(j - 1)],
        work_points[static_cast<size_t>(j)],
        clampValue(alpha, 0.0, 1.0));
    }
  }

  return work_points[degree_];
}

size_t CubicBSpline2D::findKnotSpan(double u) const
{
  if (u >= 1.0) {
    return control_points_.size() - 1;
  }

  size_t low = degree_;
  size_t high = control_points_.size();
  size_t mid = (low + high) / 2;
  while (u < knots_[mid] || u >= knots_[mid + 1]) {
    if (u < knots_[mid]) {
      high = mid;
    } else {
      low = mid;
    }
    mid = (low + high) / 2;
  }
  return mid;
}

std::vector<double> CubicBSpline2D::buildClampedUniformKnots(
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

Point2D CubicBSpline2D::interpolate(const Point2D & a, const Point2D & b, double t)
{
  return Point2D {
    a.x + (b.x - a.x) * t,
    a.y + (b.y - a.y) * t};
}

double CubicBSpline2D::distance(const Point2D & a, const Point2D & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double CubicBSpline2D::clampValue(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

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

void BSplinePathOptimizer::setObstacleCostmap(
  const std::shared_ptr<nav2_costmap_2d::Costmap2D> & costmap)
{
  obstacle_costmap_ = costmap;
}

void BSplinePathOptimizer::clearObstacleCostmap()
{
  obstacle_costmap_.reset();
}

void BSplinePathOptimizer::setEsdfProvider(const EsdfProviderPtr & provider)
{
  esdf_provider_ = provider;
}

void BSplinePathOptimizer::clearEsdfProvider()
{
  esdf_provider_.reset();
}

OptimizationResult BSplinePathOptimizer::optimizeDetailed(
  const nav_msgs::msg::Path & input_path) const
{
  OptimizationResult result;
  const auto raw_points = extractPolyline(input_path);
  const auto filtered_points = filterClosePoints(raw_points);
  std::vector<Point2D> final_points;

  if (filtered_points.size() < static_cast<size_t>(std::max(2, params_.min_control_points))) {
    final_points = filtered_points;
  } else {
    final_points = buildSmoothedPolyline(filtered_points);
  }

  result.path = buildPathMessage(input_path.header, final_points);
  result.profile = buildTrajectoryProfile(final_points);
  return result;
}

nav_msgs::msg::Path BSplinePathOptimizer::optimize(const nav_msgs::msg::Path & input_path) const
{
  return optimizeDetailed(input_path).path;
}

TrajectoryProfile2D BSplinePathOptimizer::evaluateProfile(
  const nav_msgs::msg::Path & path) const
{
  return buildTrajectoryProfile(filterClosePoints(extractPolyline(path)));
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

CubicBSpline2D BSplinePathOptimizer::buildSpline(const std::vector<Point2D> & points) const
{
  return CubicBSpline2D(points, params_.derivative_step);
}

std::vector<Point2D> BSplinePathOptimizer::sampleSplineDense(const CubicBSpline2D & spline) const
{
  if (!spline.valid()) {
    return {};
  }

  const double total_length = spline.totalLength();
  const size_t sample_count = std::max<size_t>(
    2, static_cast<size_t>(std::ceil(total_length / params_.output_path_spacing)) + 1);
  std::vector<Point2D> points;
  points.reserve(sample_count);
  for (size_t i = 0; i < sample_count; ++i) {
    const double s = (sample_count == 1) ? 0.0 :
      total_length * static_cast<double>(i) / static_cast<double>(sample_count - 1);
    points.push_back(spline.getPoint(s));
  }
  return points;
}

std::vector<Point2D> BSplinePathOptimizer::refinePathForCurvature(
  const std::vector<Point2D> & dense_points,
  const std::vector<Point2D> & reference) const
{
  if (dense_points.size() < 3 || params_.curvature_refinement_iterations <= 0) {
    return dense_points;
  }

  std::vector<Point2D> refined = dense_points;
  for (int iter = 0; iter < params_.curvature_refinement_iterations; ++iter) {
    std::vector<Point2D> next = refined;
    const double ds = std::max(params_.output_path_spacing, kEpsilon);
    for (size_t i = 1; i + 1 < refined.size(); ++i) {
      const Point2D first = discreteFirstDerivative(refined, i, ds);
      const Point2D second = discreteSecondDerivative(refined, i, ds);
      const double curvature = computeCurvature(first, second);
      const double violation = std::max(0.0, std::abs(curvature) - params_.curvature_limit);
      if (violation <= 0.0) {
        continue;
      }

      Point2D blended {
        0.5 * (refined[i - 1].x + refined[i + 1].x),
        0.5 * (refined[i - 1].y + refined[i + 1].y)};

      const double gain = params_.curvature_refinement_gain * violation;
      Point2D candidate {
        refined[i].x + (blended.x - refined[i].x) * gain,
        refined[i].y + (blended.y - refined[i].y) * gain};
      next[i] = clampToCorridor(candidate, reference);
    }
    refined.swap(next);
  }

  return refined;
}

std::vector<Point2D> BSplinePathOptimizer::buildSmoothedPolyline(
  const std::vector<Point2D> & points) const
{
  const auto control_points = resamplePolyline(points, params_.control_point_spacing);
  if (control_points.size() < std::max(kSplineDegree + 1, static_cast<size_t>(params_.min_control_points))) {
    return resamplePolyline(points, params_.output_path_spacing);
  }

  const auto spline = buildSpline(control_points);
  if (!spline.valid()) {
    return resamplePolyline(points, params_.output_path_spacing);
  }

  auto dense_points = sampleSplineDense(spline);
  dense_points = refinePathUnified(dense_points, points);
  dense_points.front() = points.front();
  dense_points.back() = points.back();
  return resamplePolyline(dense_points, params_.output_path_spacing);
}

TrajectoryProfile2D BSplinePathOptimizer::buildTrajectoryProfile(
  const std::vector<Point2D> & points) const
{
  TrajectoryProfile2D profile;
  if (points.size() < 2) {
    return profile;
  }

  profile.samples.resize(points.size());
  std::vector<double> arc_lengths(points.size(), 0.0);
  for (size_t i = 1; i < points.size(); ++i) {
    arc_lengths[i] = arc_lengths[i - 1] + distance(points[i - 1], points[i]);
  }
  profile.total_length = arc_lengths.back();

  const double ds_default = std::max(params_.output_path_spacing, kEpsilon);
  for (size_t i = 0; i < points.size(); ++i) {
    const double ds = (i == 0) ? std::max(kEpsilon, arc_lengths[1] - arc_lengths[0]) :
      (i + 1 == points.size() ? std::max(kEpsilon, arc_lengths[i] - arc_lengths[i - 1]) :
      std::max(kEpsilon, 0.5 * (arc_lengths[i + 1] - arc_lengths[i - 1])));
    const Point2D first = discreteFirstDerivative(points, i, ds_default);
    const Point2D second = discreteSecondDerivative(points, i, ds_default);
    const double curvature = computeCurvature(first, second);
    const double curvature_violation = std::max(0.0, std::abs(curvature) - params_.curvature_limit);

    auto & sample = profile.samples[i];
    sample.s = arc_lengths[i];
    sample.point = points[i];
    sample.first_derivative = first;
    sample.second_derivative = second;
    sample.curvature = curvature;
    sample.speed_limit = params_.global_speed_limit;
    sample.speed = params_.global_speed_limit;
    sample.acceleration = 0.0;

    profile.max_abs_curvature = std::max(profile.max_abs_curvature, std::abs(curvature));
    profile.curvature_penalty += curvature_violation * curvature_violation;
    (void)ds;
  }

  profile.curvature_penalty *= params_.curvature_weight;
  applyCurvatureSpeedLimits(profile);
  applyAccelerationLimits(profile);
  smoothVelocityProfile(profile);

  profile.obstacle_cost = 0.0;
  for (auto & sample : profile.samples) {
    double esdf_distance = 0.0;
    unsigned char obstacle_cost = 0;
    if (params_.use_esdf_obstacle_cost && sampleEsdfDistance(sample.point, esdf_distance)) {
      profile.obstacle_cost += computeObstaclePenaltyFromDistance(esdf_distance);
    } else if (sampleObstacleCost(sample.point, obstacle_cost)) {
      profile.obstacle_cost += computeObstaclePenalty(obstacle_cost);
    }
  }
  profile.obstacle_cost *= params_.obstacle_weight;

  profile.total_time = 0.0;
  for (size_t i = 1; i < profile.samples.size(); ++i) {
    const double ds = arc_lengths[i] - arc_lengths[i - 1];
    const double avg_v = std::max(
      1e-3, 0.5 * (profile.samples[i].speed + profile.samples[i - 1].speed));
    const double dt = ds / avg_v;
    profile.total_time += dt;
    profile.samples[i].t = profile.total_time;
    profile.samples[i].acceleration =
      (profile.samples[i].speed - profile.samples[i - 1].speed) / std::max(dt, 1e-3);
  }

  profile.total_cost =
    profile.curvature_penalty +
    profile.velocity_smoothness_cost +
    profile.obstacle_cost;

  return profile;
}

std::vector<Point2D> BSplinePathOptimizer::refinePathUnified(
  const std::vector<Point2D> & dense_points,
  const std::vector<Point2D> & reference) const
{
  if (dense_points.size() < 3) {
    return dense_points;
  }

  std::vector<Point2D> refined = dense_points;
  const int total_iterations = std::max(
    params_.curvature_refinement_iterations,
    params_.obstacle_refinement_iterations);
  const double ds = std::max(params_.output_path_spacing, kEpsilon);

  for (int iter = 0; iter < total_iterations; ++iter) {
    std::vector<Point2D> next = refined;
    for (size_t i = 1; i + 1 < refined.size(); ++i) {
      Point2D candidate = refined[i];

      if (iter < params_.curvature_refinement_iterations) {
        const Point2D first = discreteFirstDerivative(refined, i, ds);
        const Point2D second = discreteSecondDerivative(refined, i, ds);
        const double curvature = computeCurvature(first, second);
        const double violation = std::max(0.0, std::abs(curvature) - params_.curvature_limit);
        if (violation > 0.0) {
          Point2D blended {
            0.5 * (refined[i - 1].x + refined[i + 1].x),
            0.5 * (refined[i - 1].y + refined[i + 1].y)};
          const double gain = params_.curvature_refinement_gain * violation;
          candidate.x += (blended.x - candidate.x) * gain;
          candidate.y += (blended.y - candidate.y) * gain;
        }
      }

      if (iter < params_.obstacle_refinement_iterations) {
        double obstacle_penalty = 0.0;
        Point2D gradient;
        double esdf_distance = 0.0;
        unsigned char obstacle_cost = 0;
        if (params_.use_esdf_obstacle_cost && sampleEsdfDistance(candidate, esdf_distance)) {
          obstacle_penalty = computeObstaclePenaltyFromDistance(esdf_distance);
          gradient = estimateEsdfGradient(candidate);
        } else if (sampleObstacleCost(candidate, obstacle_cost)) {
          obstacle_penalty = computeObstaclePenalty(obstacle_cost);
          gradient = estimateObstacleGradient(candidate);
        }

        if (obstacle_penalty > 0.0) {
          const double gain = std::min(
            params_.max_lateral_deviation * 0.35,
            params_.obstacle_refinement_gain * obstacle_penalty);
          candidate.x += gradient.x * gain;
          candidate.y += gradient.y * gain;
        }
      }

      next[i] = clampToCorridor(candidate, reference);
    }
    refined.swap(next);
  }

  return refined;
}

void BSplinePathOptimizer::applyCurvatureSpeedLimits(TrajectoryProfile2D & profile) const
{
  for (auto & sample : profile.samples) {
    const double abs_curvature = std::abs(sample.curvature);
    if (abs_curvature <= 1e-6) {
      sample.speed_limit = params_.global_speed_limit;
    } else {
      sample.speed_limit = std::min(
        params_.global_speed_limit,
        std::sqrt(params_.lateral_accel_limit / abs_curvature));
    }
    sample.speed = sample.speed_limit;
  }
}

void BSplinePathOptimizer::applyAccelerationLimits(TrajectoryProfile2D & profile) const
{
  if (profile.samples.size() < 2) {
    return;
  }

  std::vector<double> arc_lengths(profile.samples.size(), 0.0);
  for (size_t i = 0; i < profile.samples.size(); ++i) {
    arc_lengths[i] = profile.samples[i].s;
  }

  for (size_t i = 1; i < profile.samples.size(); ++i) {
    const double ds = std::max(kEpsilon, arc_lengths[i] - arc_lengths[i - 1]);
    const double prev_v = profile.samples[i - 1].speed;
    const double reachable = std::sqrt(
      std::max(0.0, prev_v * prev_v + 2.0 * params_.longitudinal_accel_limit * ds));
    profile.samples[i].speed = std::min(profile.samples[i].speed, reachable);
  }

  for (size_t i = profile.samples.size() - 1; i > 0; --i) {
    const double ds = std::max(kEpsilon, arc_lengths[i] - arc_lengths[i - 1]);
    const double next_v = profile.samples[i].speed;
    const double reachable = std::sqrt(
      std::max(0.0, next_v * next_v + 2.0 * params_.longitudinal_accel_limit * ds));
    profile.samples[i - 1].speed = std::min(profile.samples[i - 1].speed, reachable);
  }
}

void BSplinePathOptimizer::smoothVelocityProfile(TrajectoryProfile2D & profile) const
{
  if (profile.samples.size() < 3 || params_.velocity_smoothing_gain <= 0.0) {
    return;
  }

  std::vector<double> smoothed(profile.samples.size(), 0.0);
  for (size_t i = 0; i < profile.samples.size(); ++i) {
    smoothed[i] = profile.samples[i].speed;
  }

  for (int iter = 0; iter < 3; ++iter) {
    for (size_t i = 1; i + 1 < smoothed.size(); ++i) {
      const double blended = 0.5 * (smoothed[i - 1] + smoothed[i + 1]);
      const double candidate =
        smoothed[i] + (blended - smoothed[i]) * params_.velocity_smoothing_gain;
      smoothed[i] = std::min(profile.samples[i].speed_limit, std::max(0.0, candidate));
    }
  }

  profile.velocity_smoothness_cost = 0.0;
  for (size_t i = 1; i < smoothed.size(); ++i) {
    const double dv = smoothed[i] - smoothed[i - 1];
    profile.velocity_smoothness_cost += dv * dv;
  }

  for (size_t i = 0; i < smoothed.size(); ++i) {
    profile.samples[i].speed = smoothed[i];
  }
}

Point2D BSplinePathOptimizer::discreteFirstDerivative(
  const std::vector<Point2D> & points, size_t index, double ds)
{
  const double safe_ds = std::max(ds, kEpsilon);
  if (index == 0) {
    return Point2D {
      (points[1].x - points[0].x) / safe_ds,
      (points[1].y - points[0].y) / safe_ds};
  }
  if (index + 1 == points.size()) {
    return Point2D {
      (points[index].x - points[index - 1].x) / safe_ds,
      (points[index].y - points[index - 1].y) / safe_ds};
  }
  return Point2D {
    (points[index + 1].x - points[index - 1].x) / (2.0 * safe_ds),
    (points[index + 1].y - points[index - 1].y) / (2.0 * safe_ds)};
}

Point2D BSplinePathOptimizer::discreteSecondDerivative(
  const std::vector<Point2D> & points, size_t index, double ds)
{
  const double safe_ds = std::max(ds, kEpsilon);
  if (index == 0 || index + 1 == points.size()) {
    return {};
  }
  const double inv = 1.0 / (safe_ds * safe_ds);
  return Point2D {
    (points[index + 1].x - 2.0 * points[index].x + points[index - 1].x) * inv,
    (points[index + 1].y - 2.0 * points[index].y + points[index - 1].y) * inv};
}

double BSplinePathOptimizer::computeCurvature(
  const Point2D & first_derivative,
  const Point2D & second_derivative)
{
  const double numerator =
    first_derivative.x * second_derivative.y -
    first_derivative.y * second_derivative.x;
  const double denom_sq =
    first_derivative.x * first_derivative.x +
    first_derivative.y * first_derivative.y;
  if (denom_sq <= kEpsilon) {
    return 0.0;
  }
  return numerator / std::pow(denom_sq, 1.5);
}

bool BSplinePathOptimizer::sampleObstacleCost(
  const Point2D & point,
  unsigned char & cost) const
{
  if (!obstacle_costmap_) {
    return false;
  }
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!obstacle_costmap_->worldToMap(point.x, point.y, mx, my)) {
    return false;
  }
  cost = obstacle_costmap_->getCost(mx, my);
  return true;
}

bool BSplinePathOptimizer::sampleEsdfDistance(
  const Point2D & point,
  double & distance) const
{
  if (!esdf_provider_ || !esdf_provider_->available()) {
    return false;
  }

  distance = esdf_provider_->getDistance(point.x, point.y);
  return std::isfinite(distance) && distance >= 0.0;
}

Point2D BSplinePathOptimizer::estimateObstacleGradient(
  const Point2D & point) const
{
  if (!obstacle_costmap_) {
    return {};
  }

  const double step = std::max(params_.output_path_spacing, 0.05);
  unsigned char cx = 0;
  unsigned char mx = 0;
  unsigned char px = 0;
  unsigned char my = 0;
  unsigned char py = 0;
  sampleObstacleCost(point, cx);
  sampleObstacleCost(Point2D {point.x - step, point.y}, mx);
  sampleObstacleCost(Point2D {point.x + step, point.y}, px);
  sampleObstacleCost(Point2D {point.x, point.y - step}, my);
  sampleObstacleCost(Point2D {point.x, point.y + step}, py);

  Point2D gradient {
    static_cast<double>(mx) - static_cast<double>(px),
    static_cast<double>(my) - static_cast<double>(py)};
  const double norm = std::max(kEpsilon, std::sqrt(gradient.x * gradient.x + gradient.y * gradient.y));
  gradient.x /= norm;
  gradient.y /= norm;
  return gradient;
}

Point2D BSplinePathOptimizer::estimateEsdfGradient(
  const Point2D & point) const
{
  if (!esdf_provider_ || !esdf_provider_->available()) {
    return {};
  }

  const Eigen::Vector2d gradient = esdf_provider_->getGradient(point.x, point.y);
  if (!std::isfinite(gradient.x()) || !std::isfinite(gradient.y())) {
    return {};
  }

  const double norm = std::max(kEpsilon, gradient.norm());
  return Point2D {gradient.x() / norm, gradient.y() / norm};
}

double BSplinePathOptimizer::computeObstaclePenalty(unsigned char cost) const
{
  if (cost <= params_.obstacle_safe_cost) {
    return 0.0;
  }
  const double violation =
    static_cast<double>(cost - params_.obstacle_safe_cost) /
    static_cast<double>(std::max(1, 255 - static_cast<int>(params_.obstacle_safe_cost)));
  return violation * violation;
}

double BSplinePathOptimizer::computeObstaclePenaltyFromDistance(double distance) const
{
  if (distance >= params_.obstacle_safe_distance) {
    return 0.0;
  }

  const double violation = std::max(0.0, params_.obstacle_safe_distance - distance);
  return violation * violation;
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
