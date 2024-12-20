
#include "RadarImuVelResidual.hpp"

#include <math.h>
#include <ros/ros.h>

#include "MathUtility.hpp"

namespace Residuals {
RadarImuVelocityResidual::RadarImuVelocityResidual(
    Eigen::Vector3d angularVelocity, double doppler, double sqrtInfoGain)
    : angularVel(angularVelocity),
      dopplerVel(doppler),
      sqrtInfoGain(sqrtInfoGain) {}

RadarImuVelocityResidual::RadarImuVelocityResidual(
    Eigen::Vector3d angularVelocity, double doppler, double azimuth,
    double elevation, double SigmaTheta, double SigmaPhi, double maxInfoGain)
    : angularVel(angularVelocity),
      dopplerVel(doppler),
      Theta(azimuth),
      Phi(elevation),
      SigmaTheta(SigmaTheta),
      SigmaPhi(SigmaPhi),
      maxInfoGain(maxInfoGain) {
  JThetaPhi << -sin(Theta) * cos(Phi), -cos(Theta) * sin(Phi),
      cos(Theta) * cos(Phi), -sin(Theta) * sin(Phi), 0, cos(Phi);
}

double RadarImuVelocityResidual::calculateSqrtInfoGain(
    Eigen::Vector3d velInRadar) const {
  double sqrtInfoGain;

  Eigen::Matrix<double, 2, 2> Sigma;
  Sigma << SigmaTheta * SigmaTheta, 0, 0, SigmaPhi * SigmaPhi;
  sqrtInfoGain = 1.0 / (velInRadar.transpose() * JThetaPhi * Sigma *
                        JThetaPhi.transpose() * velInRadar);

  sqrtInfoGain = sqrt(sqrtInfoGain);
  sqrtInfoGain = (sqrtInfoGain > maxInfoGain || std::isnan(sqrtInfoGain))
                     ? maxInfoGain
                     : sqrtInfoGain;
  // std::cout << "sqrtInfoGain: " << sqrtInfoGain << std::endl;
  return sqrtInfoGain;
}

bool RadarImuVelocityResidual::Evaluate(double const *const *parameters,
                                        double *residuals,
                                        double **jacobians) const {
  // rotation of current frame (4)
  Eigen::Quaterniond qImuToW(parameters[0][3], parameters[0][0],
                             parameters[0][1], parameters[0][2]);
  // velocity of current frame (3)
  Eigen::Vector3d velInWorld(parameters[1][0], parameters[1][1],
                             parameters[1][2]);
  // gyroscope bias of current frame (3)
  Eigen::Vector3d gyroBias(parameters[2][0], parameters[2][1],
                           parameters[2][2]);
  // translation of extrinsic
  Eigen::Vector3d tRadarToImu(parameters[3][0], parameters[3][1],
                              parameters[3][2]);
  // rotation of extrinsic
  Eigen::Quaterniond qRadarToImu(parameters[4][3], parameters[4][0],
                                 parameters[4][1], parameters[4][2]);
  // point state in radar
  Eigen::Vector3d pointState(parameters[5][0], parameters[5][1],
                             parameters[5][2]);

  Eigen::Vector3d unbiasedAngularVel = angularVel - gyroBias;
  Eigen::Vector3d tangentVel =
      MathUtility::skewSymmetric(unbiasedAngularVel) * tRadarToImu;
  Eigen::Vector3d radialVel = qImuToW.inverse() * velInWorld;
  Eigen::Vector3d velInRadar = qRadarToImu.inverse() * (radialVel + tangentVel);
  Eigen::Vector3d direction = pointState.normalized();

  double DynamicSqrtInfoGain;
  if (sqrtInfoGain == 0) {
    DynamicSqrtInfoGain = calculateSqrtInfoGain(velInRadar);
  } else {
    DynamicSqrtInfoGain = sqrtInfoGain;
  }

  // ROS_INFO("DynamicSqrtInfoGain: %f", DynamicSqrtInfoGain);

  *residuals = DynamicSqrtInfoGain * (direction.dot(velInRadar) - dopplerVel);

  if (jacobians == nullptr) return true;

  Eigen::Matrix3d rIToW = qImuToW.toRotationMatrix();
  Eigen::Matrix3d rRtoI = qRadarToImu.toRotationMatrix();

  // jacobian w.r.t to current rotation
  if (jacobians[0] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobianRotation(
        jacobians[0]);
    jacobianRotation.setZero();
    jacobianRotation.block<1, 3>(0, 0) = direction.transpose() *
                                         rRtoI.transpose() *
                                         MathUtility::skewSymmetric(radialVel);
    jacobianRotation = DynamicSqrtInfoGain * jacobianRotation;
  }

  // jacobian w.r.t to current velocity
  if (jacobians[1] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobianVelocity(
        jacobians[1]);
    jacobianVelocity.setZero();
    jacobianVelocity =
        direction.transpose() * rRtoI.transpose() * rIToW.transpose();
    jacobianVelocity = DynamicSqrtInfoGain * jacobianVelocity;
  }

  // jacobian w.r.t to current gyroscope bias
  if (jacobians[2] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobianGyroBias(
        jacobians[2]);
    jacobianGyroBias.setZero();
    jacobianGyroBias = direction.transpose() * rRtoI.transpose() *
                       MathUtility::skewSymmetric(tRadarToImu);
    jacobianGyroBias = DynamicSqrtInfoGain * jacobianGyroBias;
  }

  // jacobian w.r.t to extrinsic translation
  if (jacobians[3] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobianPositionEx(
        jacobians[3]);
    jacobianPositionEx.setZero();
    jacobianPositionEx = direction.transpose() * rRtoI.transpose() *
                         MathUtility::skewSymmetric(unbiasedAngularVel);
    jacobianPositionEx = DynamicSqrtInfoGain * jacobianPositionEx;
  }

  // jacobian w.r.t to extrinsic rotation
  if (jacobians[4] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobianRotationEx(
        jacobians[4]);
    jacobianRotationEx.setZero();
    jacobianRotationEx.block<1, 3>(0, 0) =
        direction.transpose() * MathUtility::skewSymmetric(velInRadar);
    jacobianRotationEx = DynamicSqrtInfoGain * jacobianRotationEx;
  }

  // jacobian w.r.t to Point State
  if (jacobians[5] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobianPointState(
        jacobians[5]);
    double normValue = pointState.norm();
    jacobianPointState =
        DynamicSqrtInfoGain *
        (velInRadar.transpose() / normValue -
         pointState.transpose() * velInRadar * pointState.transpose() /
             (normValue * normValue * normValue));
    // jacobianPositionEx = -direction.transpose() * rRtoI.transpose() *
    //                      MathUtility::skewSymmetric(unbiasedAngularVel);
    // jacobianPositionEx = sqrtInfoGain * jacobianPositionEx;
  }
  return true;
}

}  // namespace Residuals