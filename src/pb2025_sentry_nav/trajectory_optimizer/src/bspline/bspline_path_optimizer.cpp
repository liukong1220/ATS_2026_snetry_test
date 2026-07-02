// Copyright 2026

#include "trajectory_optimizer/bspline/bspline_path_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace trajectory_optimizer
{

namespace
{

constexpr size_t kSplineDegree = 3;
constexpr double kEpsilon = 1e-9;
constexpr size_t kArcLengthSamples = 400;

double pointDot(const Point2D & a, const Point2D & b)
{
  return a.x * b.x + a.y * b.y;
}

void addScaled(Point2D & target, const Point2D & value, double scale)
{
  target.x += value.x * scale;
  target.y += value.y * scale;
}

std::vector<double> scaledVector(const std::vector<double> & values, double scale)
{
  std::vector<double> result(values.size(), 0.0);
  for (size_t i = 0; i < values.size(); ++i) {
    result[i] = values[i] * scale;
  }
  return result;
}

std::vector<double> addVector(
  const std::vector<double> & a,
  const std::vector<double> & b,
  double b_scale = 1.0)
{
  std::vector<double> result(a.size(), 0.0);
  for (size_t i = 0; i < a.size(); ++i) {
    result[i] = a[i] + b[i] * b_scale;
  }
  return result;
}

double dotVector(const std::vector<double> & a, const std::vector<double> & b)
{
  double value = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    value += a[i] * b[i];
  }
  return value;
}

double normVector(const std::vector<double> & values)
{
  return std::sqrt(dotVector(values, values));
}

Point2D thirdDifference(
  const std::vector<Point2D> & points,
  size_t index)
{
  return Point2D {
    points[index + 3].x - 3.0 * points[index + 2].x + 3.0 * points[index + 1].x - points[index].x,
    points[index + 3].y - 3.0 * points[index + 2].y + 3.0 * points[index + 1].y - points[index].y};
}

size_t firstEditableIndex(const std::vector<Point2D> & points)
{
  return points.size() > 6 ? 3u : 1u;
}

size_t lastEditableExclusive(const std::vector<Point2D> & points)
{
  return points.size() > 6 ? points.size() - 3u : points.size() - 1u;
}

std::vector<double> packEditableControlPoints(const std::vector<Point2D> & points)
{
  std::vector<double> values;
  for (size_t i = firstEditableIndex(points); i < lastEditableExclusive(points); ++i) {
    values.push_back(points[i].x);
    values.push_back(points[i].y);
  }
  return values;
}

void unpackEditableControlPoints(
  const std::vector<double> & values,
  std::vector<Point2D> & points)
{
  size_t cursor = 0;
  for (size_t i = firstEditableIndex(points); i < lastEditableExclusive(points); ++i) {
    points[i].x = values[cursor++];
    points[i].y = values[cursor++];
  }
}

std::vector<double> packEditableGradient(
  const std::vector<Point2D> & points,
  const std::vector<Point2D> & gradient)
{
  std::vector<double> values;
  for (size_t i = firstEditableIndex(points); i < lastEditableExclusive(points); ++i) {
    values.push_back(gradient[i].x);
    values.push_back(gradient[i].y);
  }
  return values;
}

std::vector<double> lbfgsDirection(
  const std::vector<double> & gradient,
  const std::vector<std::vector<double>> & s_history,
  const std::vector<std::vector<double>> & y_history)
{
  if (s_history.empty()) {
    return scaledVector(gradient, -1.0);
  }

  std::vector<double> q = gradient;
  std::vector<double> alpha(s_history.size(), 0.0);
  std::vector<double> rho(s_history.size(), 0.0);
  for (int i = static_cast<int>(s_history.size()) - 1; i >= 0; --i) {
    const double sy = dotVector(s_history[static_cast<size_t>(i)], y_history[static_cast<size_t>(i)]);
    rho[static_cast<size_t>(i)] = sy > 0.0 ? 1.0 / sy : 0.0;
    alpha[static_cast<size_t>(i)] =
      rho[static_cast<size_t>(i)] * dotVector(s_history[static_cast<size_t>(i)], q);
    q = addVector(q, y_history[static_cast<size_t>(i)], -alpha[static_cast<size_t>(i)]);
  }

  const auto & last_s = s_history.back();
  const auto & last_y = y_history.back();
  const double yy = dotVector(last_y, last_y);
  const double gamma = yy > 0.0 ? dotVector(last_s, last_y) / yy : 1.0;
  std::vector<double> r = scaledVector(q, gamma);

  for (size_t i = 0; i < s_history.size(); ++i) {
    const double beta = rho[i] * dotVector(y_history[i], r);
    r = addVector(r, s_history[i], alpha[i] - beta);
  }

  return scaledVector(r, -1.0);
}

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
  auto control_points = resamplePolyline(points, params_.control_point_spacing);
  if (control_points.size() < std::max(kSplineDegree + 1, static_cast<size_t>(params_.min_control_points))) {
    return resamplePolyline(points, params_.output_path_spacing);
  }

  if (params_.use_continuous_optimization) {
    const auto optimized = optimizeControlPointsContinuous(control_points, points);
    if (optimized.success && optimized.control_points.size() >= control_points.size()) {
      control_points = optimized.control_points;
    }
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

BSplinePathOptimizer::ContinuousOptimizeResult
BSplinePathOptimizer::optimizeControlPointsContinuous(
  const std::vector<Point2D> & warm_control_points,
  const std::vector<Point2D> & reference) const
{
  ContinuousOptimizeResult result;
  result.control_points = warm_control_points;

  if (warm_control_points.size() < std::max(kSplineDegree + 1, static_cast<size_t>(params_.min_control_points))) {
    return result;
  }

  std::vector<Point2D> gradient;
  const auto initial =
    evaluateContinuousCost(result.control_points, warm_control_points, reference, &gradient);
  result.initial_cost = initial.total;

  std::vector<double> x = packEditableControlPoints(result.control_points);
  std::vector<double> grad = packEditableGradient(result.control_points, gradient);
  if (x.empty()) {
    result.success = true;
    result.final_cost = result.initial_cost;
    return result;
  }

  std::vector<std::vector<double>> s_history;
  std::vector<std::vector<double>> y_history;
  ContinuousCostBreakdown current = initial;

  for (int iteration = 0; iteration < params_.continuous_max_iterations; ++iteration) {
    result.iterations = iteration + 1;
    if (normVector(grad) < params_.continuous_gradient_tolerance) {
      break;
    }

    std::vector<double> direction = lbfgsDirection(grad, s_history, y_history);
    if (dotVector(direction, grad) >= 0.0) {
      direction = scaledVector(grad, -1.0);
    }

    const double directional_derivative = dotVector(grad, direction);
    double step = params_.continuous_initial_step;
    bool accepted = false;
    std::vector<double> next_x;
    std::vector<Point2D> next_points;
    std::vector<Point2D> next_gradient;
    ContinuousCostBreakdown next_cost;

    for (int line_search = 0; line_search < 20; ++line_search) {
      next_x = addVector(x, direction, step);
      next_points = result.control_points;
      unpackEditableControlPoints(next_x, next_points);
      next_cost =
        evaluateContinuousCost(next_points, warm_control_points, reference, &next_gradient);
      if (std::isfinite(next_cost.total) &&
        next_cost.total <= current.total + 1e-4 * step * directional_derivative)
      {
        accepted = true;
        break;
      }
      step *= 0.5;
    }

    if (!accepted) {
      break;
    }

    const std::vector<double> next_grad = packEditableGradient(next_points, next_gradient);
    const std::vector<double> s = addVector(next_x, x, -1.0);
    const std::vector<double> y = addVector(next_grad, grad, -1.0);
    if (dotVector(s, y) > 1e-10) {
      s_history.push_back(s);
      y_history.push_back(y);
      if (s_history.size() > static_cast<size_t>(params_.continuous_lbfgs_memory)) {
        s_history.erase(s_history.begin());
        y_history.erase(y_history.begin());
      }
    }

    x = std::move(next_x);
    grad = next_grad;
    result.control_points = std::move(next_points);
    current = next_cost;
  }

  result.final_cost = current.total;
  result.success = std::isfinite(result.final_cost) && result.final_cost <= result.initial_cost + 1e-6;
  return result;
}

BSplinePathOptimizer::ContinuousCostBreakdown
BSplinePathOptimizer::evaluateContinuousCost(
  const std::vector<Point2D> & control_points,
  const std::vector<Point2D> & warm_control_points,
  const std::vector<Point2D> & reference,
  std::vector<Point2D> * gradient) const
{
  ContinuousCostBreakdown cost;
  if (gradient) {
    gradient->assign(control_points.size(), {});
  }

  if (control_points.size() >= 4) {
    for (size_t i = 0; i + 3 < control_points.size(); ++i) {
      const Point2D d = thirdDifference(control_points, i);
      const double raw = pointDot(d, d);
      cost.smoothness += raw;
      if (gradient) {
        addScaled((*gradient)[i], d, -2.0 * params_.smoothness_weight);
        addScaled((*gradient)[i + 1], d, 6.0 * params_.smoothness_weight);
        addScaled((*gradient)[i + 2], d, -6.0 * params_.smoothness_weight);
        addScaled((*gradient)[i + 3], d, 2.0 * params_.smoothness_weight);
      }
    }
    cost.smoothness *= params_.smoothness_weight;
  }

  for (size_t i = 0; i < control_points.size() && i < warm_control_points.size(); ++i) {
    if (i < firstEditableIndex(control_points) || i >= lastEditableExclusive(control_points)) {
      continue;
    }
    const Point2D delta {
      control_points[i].x - warm_control_points[i].x,
      control_points[i].y - warm_control_points[i].y};
    cost.fitness += pointDot(delta, delta);
    if (gradient) {
      addScaled((*gradient)[i], delta, 2.0 * params_.fitness_weight);
    }
  }
  cost.fitness *= params_.fitness_weight;

  if (control_points.size() >= 2 && warm_control_points.size() >= 2) {
    const Point2D start_delta {
      (control_points[1].x - control_points[0].x) - (warm_control_points[1].x - warm_control_points[0].x),
      (control_points[1].y - control_points[0].y) - (warm_control_points[1].y - warm_control_points[0].y)};
    const size_t last = control_points.size() - 1;
    const Point2D end_delta {
      (control_points[last].x - control_points[last - 1].x) -
      (warm_control_points[last].x - warm_control_points[last - 1].x),
      (control_points[last].y - control_points[last - 1].y) -
      (warm_control_points[last].y - warm_control_points[last - 1].y)};
    cost.endpoint_tangent = params_.endpoint_tangent_weight *
      (pointDot(start_delta, start_delta) + pointDot(end_delta, end_delta));
    if (gradient) {
      addScaled((*gradient)[0], start_delta, -2.0 * params_.endpoint_tangent_weight);
      addScaled((*gradient)[1], start_delta, 2.0 * params_.endpoint_tangent_weight);
      addScaled((*gradient)[last - 1], end_delta, -2.0 * params_.endpoint_tangent_weight);
      addScaled((*gradient)[last], end_delta, 2.0 * params_.endpoint_tangent_weight);
    }
  }

  const CubicBSpline2D spline(control_points, params_.derivative_step);
  if (!spline.valid()) {
    cost.total = std::numeric_limits<double>::infinity();
    return cost;
  }

  const double sample_step = std::max(params_.output_path_spacing, params_.control_point_spacing * 0.5);
  const double total_length = spline.totalLength();
  const size_t sample_count = std::max<size_t>(4, static_cast<size_t>(std::ceil(total_length / sample_step)) + 1);
  for (size_t sample_index = 1; sample_index + 1 < sample_count; ++sample_index) {
    const double s = total_length * static_cast<double>(sample_index) /
      static_cast<double>(sample_count - 1);
    const Point2D sample = spline.getPoint(s);

    if (params_.use_esdf_obstacle_cost) {
      double esdf_distance = 0.0;
      if (sampleEsdfDistance(sample, esdf_distance)) {
        const double violation = params_.obstacle_safe_distance - esdf_distance;
        if (violation > 0.0) {
          cost.obstacle += violation * violation;
          if (gradient) {
            const Point2D obstacle_grad = estimateEsdfGradient(sample);
            const Point2D sample_grad {
              -2.0 * params_.obstacle_weight * violation * obstacle_grad.x,
              -2.0 * params_.obstacle_weight * violation * obstacle_grad.y};
            const size_t nearest = std::min(
              control_points.size() - 2,
              std::max<size_t>(1, static_cast<size_t>(std::lround(
                static_cast<double>(sample_index) *
                static_cast<double>(control_points.size() - 1) /
                static_cast<double>(sample_count - 1)))));
            addScaled((*gradient)[nearest], sample_grad, 1.0);
          }
        }
      }
    } else {
      unsigned char obstacle_cost = 0;
      if (sampleObstacleCost(sample, obstacle_cost)) {
        const double penalty = computeObstaclePenalty(obstacle_cost);
        cost.obstacle += penalty;
        if (gradient && penalty > 0.0) {
          const Point2D obstacle_grad = estimateObstacleGradient(sample);
          const size_t nearest = std::min(
            control_points.size() - 2,
            std::max<size_t>(1, static_cast<size_t>(std::lround(
              static_cast<double>(sample_index) *
              static_cast<double>(control_points.size() - 1) /
              static_cast<double>(sample_count - 1)))));
          addScaled((*gradient)[nearest], obstacle_grad, params_.obstacle_weight * penalty);
        }
      }
    }

    if (reference.size() >= 2) {
      const auto corridor_projection = closestPointOnPolyline(sample, reference);
      const double allowed_deviation = computeAllowedCorridorDeviation(sample);
      if (corridor_projection.second > allowed_deviation) {
        const Point2D delta {
          sample.x - corridor_projection.first.x,
          sample.y - corridor_projection.first.y};
        const double excess = corridor_projection.second - allowed_deviation;
        cost.corridor += excess * excess;
        if (gradient) {
          const double inv = 1.0 / std::max(kEpsilon, corridor_projection.second);
          const Point2D direction {delta.x * inv, delta.y * inv};
          const Point2D corridor_grad {
            2.0 * params_.corridor_weight * excess * direction.x,
            2.0 * params_.corridor_weight * excess * direction.y};
          const size_t nearest = std::min(
            control_points.size() - 2,
            std::max<size_t>(1, static_cast<size_t>(std::lround(
              static_cast<double>(sample_index) *
              static_cast<double>(control_points.size() - 1) /
              static_cast<double>(sample_count - 1)))));
          addScaled((*gradient)[nearest], corridor_grad, 1.0);
        }
      }
    }
  }

  cost.obstacle *= params_.obstacle_weight;
  cost.corridor *= params_.corridor_weight;
  cost.total =
    cost.smoothness + cost.obstacle + cost.fitness + cost.corridor + cost.endpoint_tangent;
  return cost;
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
    sample.slope_degrees = 0.0;
    sample.speed_limit = params_.global_speed_limit;
    sample.speed = params_.global_speed_limit;
    sample.acceleration = 0.0;

    // 任务2：把 slope_grid 语义直接写进轨迹 profile，
    // 后面的限速、限加速度、governor 都从这一份统一参考出发。
    double slope_degrees = 0.0;
    if (sampleSlopeDegrees(sample.point, slope_degrees) && std::isfinite(slope_degrees)) {
      sample.slope_degrees = std::max(0.0, slope_degrees);
    }

    profile.max_abs_curvature = std::max(profile.max_abs_curvature, std::abs(curvature));
    profile.curvature_penalty += curvature_violation * curvature_violation;
    (void)ds;
  }

  profile.curvature_penalty *= params_.curvature_weight;
  applyCurvatureSpeedLimits(profile);
  applySlopeSpeedLimits(profile);
  applyObstacleSpeedLimits(profile);
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

  if (params_.use_esdf_obstacle_cost && esdf_provider_ && esdf_provider_->available()) {
    refined = refinePathSecondStageEsdf(refined, reference);
  }

  return refined;
}

std::vector<Point2D> BSplinePathOptimizer::refinePathSecondStageEsdf(
  const std::vector<Point2D> & dense_points,
  const std::vector<Point2D> & reference) const
{
  if (dense_points.size() < 3) {
    return dense_points;
  }

  static constexpr int kSecondStageIterations = 2;
  static constexpr double kTangentialRetention = 0.15;
  std::vector<Point2D> refined = dense_points;

  for (int iter = 0; iter < kSecondStageIterations; ++iter) {
    std::vector<Point2D> next = refined;
    for (size_t i = 1; i + 1 < refined.size(); ++i) {
      Point2D candidate = refined[i];
      double esdf_distance = 0.0;
      if (!sampleEsdfDistance(candidate, esdf_distance)) {
        continue;
      }

      const double obstacle_penalty = computeObstaclePenaltyFromDistance(esdf_distance);
      const Point2D gradient = estimateEsdfGradient(candidate);
      const Point2D tangent = normalizeVector(Point2D {
          refined[i + 1].x - refined[i - 1].x,
          refined[i + 1].y - refined[i - 1].y});
      Point2D filtered_gradient = gradient;
      if (std::abs(tangent.x) > kEpsilon || std::abs(tangent.y) > kEpsilon) {
        const double tangential_component = dot(gradient, tangent);
        filtered_gradient.x -= tangential_component * tangent.x * (1.0 - kTangentialRetention);
        filtered_gradient.y -= tangential_component * tangent.y * (1.0 - kTangentialRetention);
      }
      filtered_gradient = normalizeVector(filtered_gradient);

      Point2D midpoint_pull {
        0.5 * (refined[i - 1].x + refined[i + 1].x) - candidate.x,
        0.5 * (refined[i - 1].y + refined[i + 1].y) - candidate.y};

      const double obstacle_gain = std::min(
        params_.max_lateral_deviation * 0.20,
        params_.obstacle_refinement_gain * 0.50 * obstacle_penalty);
      const double shape_gain = params_.curvature_refinement_gain * 0.35;

      if (obstacle_penalty > 0.0) {
        candidate.x += filtered_gradient.x * obstacle_gain;
        candidate.y += filtered_gradient.y * obstacle_gain;
      }
      candidate.x += midpoint_pull.x * shape_gain;
      candidate.y += midpoint_pull.y * shape_gain;

      next[i] = clampToCorridor(candidate, reference);
    }
    refined.swap(next);
  }

  return refined;
}

void BSplinePathOptimizer::applyCurvatureSpeedLimits(TrajectoryProfile2D & profile) const
{
  // 曲率仍然是第一层几何限速：
  // 即使地面很平，横向加速度也必须处在底盘可跟踪范围内。
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

void BSplinePathOptimizer::applySlopeSpeedLimits(TrajectoryProfile2D & profile) const
{
  if (!params_.use_slope_speed_limits) {
    return;
  }

  // 坡度限速放在“曲率之后、障碍之前”：
  // 1. 曲率先决定弯道几何能跑多快
  // 2. 坡度再决定这段地形上是否该更激进或更保守
  // 3. 最后再由障碍物距离做贴边保守化
  for (auto & sample : profile.samples) {
    const double slope_scale = computeSlopeAdaptiveScale(
      sample.slope_degrees,
      params_.slope_speed_boost_start_deg,
      params_.slope_speed_obstacle_deg,
      params_.slope_speed_limit_full_deg,
      params_.slope_speed_max_scale,
      params_.slope_speed_min_scale);
    sample.speed_limit = std::min(sample.speed_limit, params_.global_speed_limit * slope_scale);
    sample.speed = std::min(sample.speed, sample.speed_limit);
  }
}

void BSplinePathOptimizer::applyObstacleSpeedLimits(TrajectoryProfile2D & profile) const
{
  const double reduction_distance = std::max(
    params_.obstacle_speed_reduction_distance,
    params_.obstacle_speed_min_distance + 1e-3);
  const double min_distance = std::max(0.01, params_.obstacle_speed_min_distance);
  const double min_scale = clampValue(params_.obstacle_speed_min_scale, 0.05, 1.0);

  for (auto & sample : profile.samples) {
    double obstacle_distance = std::numeric_limits<double>::infinity();
    bool have_distance = false;

    if (params_.use_esdf_obstacle_cost) {
      double esdf_distance = 0.0;
      if (sampleEsdfDistance(sample.point, esdf_distance) && std::isfinite(esdf_distance) && esdf_distance >= 0.0) {
        obstacle_distance = esdf_distance;
        have_distance = true;
      }
    }

    if (!have_distance) {
      unsigned char obstacle_cost = 0;
      if (sampleObstacleCost(sample.point, obstacle_cost)) {
        const double normalized_cost =
          clampValue(static_cast<double>(obstacle_cost) / 255.0, 0.0, 1.0);
        obstacle_distance =
          reduction_distance - normalized_cost * (reduction_distance - min_distance);
        have_distance = true;
      }
    }

    if (!have_distance || obstacle_distance >= reduction_distance) {
      continue;
    }

    const double clamped_distance = clampValue(obstacle_distance, min_distance, reduction_distance);
    const double ratio =
      (clamped_distance - min_distance) / std::max(kEpsilon, reduction_distance - min_distance);
    const double obstacle_scale = min_scale + ratio * (1.0 - min_scale);
    sample.speed_limit = std::min(sample.speed_limit, params_.global_speed_limit * obstacle_scale);
    sample.speed = std::min(sample.speed, sample.speed_limit);
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
    double accel_limit = params_.longitudinal_accel_limit;
    if (params_.use_slope_accel_limits) {
      // 前向传播时取相邻两点里更保守的坡度缩放，
      // 避免在进入更陡坡段前一拍还在猛加速。
      const double slope_scale = std::min(
        computeSlopeAdaptiveScale(
          profile.samples[i - 1].slope_degrees,
          params_.slope_accel_boost_start_deg,
          params_.slope_accel_obstacle_deg,
          params_.slope_accel_limit_full_deg,
          params_.slope_accel_max_scale,
          params_.slope_accel_min_scale),
        computeSlopeAdaptiveScale(
          profile.samples[i].slope_degrees,
          params_.slope_accel_boost_start_deg,
          params_.slope_accel_obstacle_deg,
          params_.slope_accel_limit_full_deg,
          params_.slope_accel_max_scale,
          params_.slope_accel_min_scale));
      accel_limit *= slope_scale;
    }
    const double reachable = std::sqrt(
      std::max(0.0, prev_v * prev_v + 2.0 * accel_limit * ds));
    profile.samples[i].speed = std::min(profile.samples[i].speed, reachable);
  }

  for (size_t i = profile.samples.size() - 1; i > 0; --i) {
    const double ds = std::max(kEpsilon, arc_lengths[i] - arc_lengths[i - 1]);
    const double next_v = profile.samples[i].speed;
    double accel_limit = params_.longitudinal_accel_limit;
    if (params_.use_slope_accel_limits) {
      const double slope_scale = std::min(
        computeSlopeAdaptiveScale(
          profile.samples[i - 1].slope_degrees,
          params_.slope_accel_boost_start_deg,
          params_.slope_accel_obstacle_deg,
          params_.slope_accel_limit_full_deg,
          params_.slope_accel_max_scale,
          params_.slope_accel_min_scale),
        computeSlopeAdaptiveScale(
          profile.samples[i].slope_degrees,
          params_.slope_accel_boost_start_deg,
          params_.slope_accel_obstacle_deg,
          params_.slope_accel_limit_full_deg,
          params_.slope_accel_max_scale,
          params_.slope_accel_min_scale));
      accel_limit *= slope_scale;
    }
    const double reachable = std::sqrt(
      std::max(0.0, next_v * next_v + 2.0 * accel_limit * ds));
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
  return std::isfinite(distance);
}

bool BSplinePathOptimizer::sampleSlopeDegrees(
  const Point2D & point,
  double & slope_degrees) const
{
  if (!esdf_provider_ || !esdf_provider_->available()) {
    return false;
  }

  slope_degrees = esdf_provider_->getSlope(point.x, point.y);
  return std::isfinite(slope_degrees);
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

Point2D BSplinePathOptimizer::normalizeVector(const Point2D & vector)
{
  const double norm = std::sqrt(vector.x * vector.x + vector.y * vector.y);
  if (norm <= kEpsilon) {
    return {};
  }
  return Point2D {vector.x / norm, vector.y / norm};
}

double BSplinePathOptimizer::dot(const Point2D & a, const Point2D & b)
{
  return a.x * b.x + a.y * b.y;
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

double BSplinePathOptimizer::computeSlopeAdaptiveScale(
  double slope_degrees,
  double boost_start_degrees,
  double obstacle_degrees,
  double full_degrees,
  double max_scale,
  double min_scale) const
{
  if (!std::isfinite(slope_degrees) || slope_degrees <= 0.0) {
    return 1.0;
  }

  const double clamped_min_scale = clampValue(min_scale, 0.05, 1.0);
  const double clamped_max_scale = std::max(1.0, max_scale);
  const double boost_start = std::max(0.0, boost_start_degrees);
  const double obstacle = std::max(boost_start + 1e-3, obstacle_degrees);
  const double full = std::max(obstacle + 1e-3, full_degrees);

  if (slope_degrees <= boost_start) {
    return clamped_max_scale;
  }

  if (slope_degrees < obstacle) {
    // 低于“坡度障碍阈值”时做加速处理：
    // 坡度越小，加速增益越大；靠近障碍阈值时，增益逐步回落到 1.0。
    const double ratio = (slope_degrees - boost_start) / std::max(kEpsilon, obstacle - boost_start);
    return clamped_max_scale - ratio * (clamped_max_scale - 1.0);
  }

  if (slope_degrees >= full) {
    return clamped_min_scale;
  }

  // 超过“坡度障碍阈值”后，逐步进入保守限速/限加速度区。
  // 这里继续保持线性规则，方便现场快速调参和交接理解。
  const double ratio = (slope_degrees - obstacle) / std::max(kEpsilon, full - obstacle);
  return 1.0 - ratio * (1.0 - clamped_min_scale);
}

double BSplinePathOptimizer::computeAllowedCorridorDeviation(const Point2D & candidate) const
{
  double allowed_deviation = params_.max_lateral_deviation;
  if (params_.use_esdf_obstacle_cost && esdf_provider_ && esdf_provider_->available()) {
    const double distance = esdf_provider_->getDistance(candidate.x, candidate.y);
    if (std::isfinite(distance) && distance > params_.obstacle_safe_distance) {
      const double clearance_bonus =
        std::max(0.0, distance - params_.obstacle_safe_distance);
      allowed_deviation += std::min(0.10, clearance_bonus * 0.30);
    }
  }
  return allowed_deviation;
}

Point2D BSplinePathOptimizer::clampToCorridor(
  const Point2D & candidate, const std::vector<Point2D> & reference) const
{
  if (params_.max_lateral_deviation <= 0.0 || reference.size() < 2) {
    return candidate;
  }

  const double allowed_deviation = computeAllowedCorridorDeviation(candidate);

  const auto closest_result = closestPointOnPolyline(candidate, reference);
  if (closest_result.second <= allowed_deviation) {
    return candidate;
  }

  const double dx = candidate.x - closest_result.first.x;
  const double dy = candidate.y - closest_result.first.y;
  const double norm = std::max(kEpsilon, std::sqrt(dx * dx + dy * dy));
  const double scale = allowed_deviation / norm;
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
