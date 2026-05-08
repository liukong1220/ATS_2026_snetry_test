// Copyright 2026

#ifndef TRAJECTORY_OPTIMIZER__ESDF_PROVIDER_HPP_
#define TRAJECTORY_OPTIMIZER__ESDF_PROVIDER_HPP_

#include <memory>

#include <Eigen/Core>

namespace trajectory_optimizer
{

class EsdfProvider
{
public:
  virtual ~EsdfProvider() = default;

  virtual bool available() const = 0;
  virtual double getDistance(double x, double y) const = 0;
  virtual Eigen::Vector2d getGradient(double x, double y) const = 0;
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
};

using EsdfProviderPtr = std::shared_ptr<EsdfProvider>;

}  // namespace trajectory_optimizer

#endif  // TRAJECTORY_OPTIMIZER__ESDF_PROVIDER_HPP_
