#pragma once

#include <ceres/ceres.h>
#include <ceres/sized_cost_function.h>

#include "../utility/MathUtility.hpp"

namespace Residuals {
// size of parameters in each parameter block.
//
// number of residuals (3),
//
// translation of i-th frame (3)
// rotation of i-th frame (4)
// translation of extrinsic (radar to imu) (3)
// rotation of extrinsic (radar to imu) (4)
// point in world (3)

class RadarPointResidual : public ceres::SizedCostFunction<3, 3, 4, 3, 4, 3> {
 public:
  RadarPointResidual(Eigen::Vector3d measurement, double sqrtInfoGainValue);
  RadarPointResidual(Eigen::Vector3d measurement, double range, double azimuth,
                     double elevation, double SigmaRange, double SigmaTheta,
                     double SigmaPhi, double maxInfoGain,
                     int observationCount = 1, int pointNum = 1);

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const final;

 private:
  Eigen::Vector3d measuredPoint;
  Eigen::Matrix<double, 3, 3> sqrtInfoGain;

  double SigmaRange;
  double SigmaTheta;
  double SigmaPhi;
  double maxInfoGain;
  double Theta;
  double Phi;
  double Range;
  int observationCount;
  int pointNum;
  Eigen::Matrix<double, 3, 3> JRangeThetaPhi;
  void calculateSqrtInfoGain(double range, double azimuth, double elevation);
};
}  // namespace Residuals