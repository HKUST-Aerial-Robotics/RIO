#include "EigenQuaternionManifold.hpp"

#include "MathUtility.hpp"

namespace Manifold {
int EigenQuaternionManifold::AmbientSize() const { return 4; }

int EigenQuaternionManifold::TangentSize() const { return 3; }

bool EigenQuaternionManifold::Plus(const double *x, const double *delta,
                                   double *x_plus_delta) const {
  Eigen::Map<const Eigen::Quaterniond> quaternionSrc(x);
  Eigen::Map<const Eigen::Vector3d> deltaVec(delta);
  Eigen::Map<Eigen::Quaterniond> quaternionDes(x_plus_delta);
  quaternionDes = (quaternionSrc * MathUtility::deltaQ(deltaVec)).normalized();

  return true;
}

bool EigenQuaternionManifold::PlusJacobian(const double *x,
                                           double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> jacobianWrapper(
      jacobian);
  jacobianWrapper.topRows<3>().setIdentity();
  jacobianWrapper.bottomRows<1>().setZero();
  return true;
}

bool EigenQuaternionManifold::Minus(const double *y, const double *x,
                                    double *y_minus_x) const {
  Eigen::Map<const Eigen::Quaterniond> quaternionSrc(y);
  Eigen::Map<const Eigen::Quaterniond> quaternionDes(x);
  Eigen::Map<Eigen::Vector3d> deltaVec(y_minus_x);
  deltaVec = MathUtility::vectorQ(quaternionSrc * quaternionDes.inverse());
  return true;
}

bool EigenQuaternionManifold::MinusJacobian(const double *x,
                                            double *jacobian) const {
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobianWrapper(
      jacobian);
  jacobianWrapper.topRows<3>().setIdentity();
  jacobianWrapper.rightCols<1>().setZero();
  jacobianWrapper = -jacobianWrapper;
  return true;
}

}  // namespace Manifold