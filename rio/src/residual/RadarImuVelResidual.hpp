#pragma once
#include <ceres/sized_cost_function.h>

#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "ceres/ceres.h"

namespace Residuals {
// size of parameters in each parameter block.
//
// number of residuals (1),
//
// rotation (4)
// velocity (3)
// gyroscope bias (3),
// extrinsic transaltion (3)
// extrinsic rotation (4)
// point state in radar (3)

class RadarImuVelocityResidual
    : public ceres::SizedCostFunction<1, 4, 3, 3, 3, 4, 3> {
 public:
  RadarImuVelocityResidual(Eigen::Vector3d angularVelocity, double doppler,
                           double sqrtInfoGain);
  RadarImuVelocityResidual(Eigen::Vector3d angularVelocity, double doppler,
                           double azimuth, double elevation, double SigmaTheta,
                           double SigmaPhi, double maxInfoGain);

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const final;

 private:
  Eigen::Vector3d angularVel;
  double dopplerVel;
  double sqrtInfoGain = 0;
  double maxInfoGain = 0;

  double Theta;
  double Phi;
  double SigmaTheta;
  double SigmaPhi;
  Eigen::Matrix<double, 3, 2> JThetaPhi;

  double calculateSqrtInfoGain(Eigen::Vector3d velInRadar) const;
};
}  // namespace Residuals