// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__ESDF_PROVIDER_HPP_
#define TRAJECTORY_OPTIMIZER__ESDF_PROVIDER_HPP_

#include <cmath>
#include <limits>
#include <memory>

#include <Eigen/Core>

namespace trajectory_optimizer
{

struct EsdfQueryResult
{
  bool valid = false;
  bool inside_local_window = false;
  double distance = std::numeric_limits<double>::quiet_NaN();
  double slope = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
};

class EsdfProvider
{
public:
  virtual ~EsdfProvider() = default;

  virtual bool available() const = 0;
  virtual double getDistance(double x, double y) const = 0;
  virtual Eigen::Vector2d getGradient(double x, double y) const = 0;
  virtual double getSlope(double, double) const
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  virtual bool isInsideLocalWindow(double, double) const
  {
    return true;
  }

  virtual bool query(double x, double y, EsdfQueryResult & result) const
  {
    result = EsdfQueryResult {};
    result.inside_local_window = isInsideLocalWindow(x, y);
    if (!available() || !result.inside_local_window) {
      return false;
    }

    result.distance = getDistance(x, y);
    result.gradient = getGradient(x, y);
    result.slope = getSlope(x, y);
    result.valid = std::isfinite(result.distance);
    return result.valid;
  }
};

class NullEsdfProvider : public EsdfProvider
{
public:
  bool available() const override
  {
    return false;
  }

  double getDistance(double, double) const override
  {
    return -1.0;
  }

  Eigen::Vector2d getGradient(double, double) const override
  {
    return Eigen::Vector2d::Zero();
  }

  double getSlope(double, double) const override
  {
    return std::numeric_limits<double>::quiet_NaN();
  }

  bool isInsideLocalWindow(double, double) const override
  {
    return false;
  }
};

using EsdfProviderPtr = std::shared_ptr<EsdfProvider>;

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__ESDF_PROVIDER_HPP_
