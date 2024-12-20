#pragma once
#include <ceres/sized_cost_function.h>

// #include <Eigen/Eigen>

#include "Preintegration.hpp"

namespace Residuals {
// size of parameters in each parameter block.
//
// number of residuals (15),
//
// position of previous imu frame (3),
// rotation of previous imu frame (4),
// velocity of previous imu frame (3),
// accelerometer bias of previous imu frame (3),
// gyroscope bias of previous imu frame (3),
//
// position of current imu frame (3),
// rotation of current imu frame (4),
// velocity of current imu frame (3),
// accelerometer bias of current imu frame (3),
// gyroscope bias of current imu frame (3)

class IMUResidual
    : public ceres::SizedCostFunction<15, 3, 4, 3, 3, 3, 3, 4, 3, 3, 3> {
 public:
  IMUResidual(Preintegration imuPreintegration, const Eigen::Vector3d &gravity);

  bool Evaluate(double const *const *parameters, double *residuals,
                double **jacobians) const final;

 private:
  Preintegration imuIntegration;
  Eigen::Vector3d gInW;
};
}  // namespace Residuals