#pragma once

#include <ceres/manifold.h>

#include "Eigen/Eigen"

namespace Manifold {
class EigenQuaternionManifold : public ceres::Manifold {
 public:
  EigenQuaternionManifold() = default;
  ~EigenQuaternionManifold() override = default;

  int AmbientSize() const final;

  int TangentSize() const final;

  bool Plus(const double *x, const double *delta,
            double *x_plus_delta) const final;

  bool PlusJacobian(const double *x, double *jacobian) const final;

  bool Minus(const double *y, const double *x, double *y_minus_x) const final;

  bool MinusJacobian(const double *x, double *jacobian) const final;
};
}  // namespace Manifold