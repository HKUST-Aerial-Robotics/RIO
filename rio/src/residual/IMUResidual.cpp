#include "IMUResidual.hpp"

#include <iostream>
#include <utility>

#include "MathUtility.hpp"
// #include <Eigen/src/Geometry/Quaternion.h>
// #include <Eigen/src/Core/Matrix.h>

namespace Residuals {
IMUResidual::IMUResidual(Preintegration imuPreintegration,
                         const Eigen::Vector3d &gravity)
    : imuIntegration(imuPreintegration), gInW(gravity){};

bool IMUResidual::Evaluate(double const *const *parameters, double *residuals,
                           double **jacobians) const {
  // position of 1st imu frame (3)
  Eigen::Vector3d tIToW(parameters[0][0], parameters[0][1], parameters[0][2]);
  // rotation of 1st imu frame (4)
  Eigen::Quaterniond qIToW(parameters[1][3], parameters[1][0], parameters[1][1],
                           parameters[1][2]);
  // velocity of 1st imu frame (3)
  Eigen::Vector3d velocityI(parameters[2][0], parameters[2][1],
                            parameters[2][2]);
  // accelerometer bias of 1st imu frame (3)
  Eigen::Vector3d biasAccI(parameters[3][0], parameters[3][1],
                           parameters[3][2]);
  // gyroscope bias of 1st imu frame (3)
  Eigen::Vector3d biasGyroI(parameters[4][0], parameters[4][1],
                            parameters[4][2]);

  // position of 2nd imu frame (3)
  Eigen::Vector3d tJToW(parameters[5][0], parameters[5][1], parameters[5][2]);
  // rotation of 2nd imu frame (4)
  Eigen::Quaterniond qJToW(parameters[6][3], parameters[6][0], parameters[6][1],
                           parameters[6][2]);
  // velocity of 2nd imu frame (3)
  Eigen::Vector3d velocityJ(parameters[7][0], parameters[7][1],
                            parameters[7][2]);
  // accelerometer bias of 2nd imu frame (3)
  Eigen::Vector3d biasAccJ(parameters[8][0], parameters[8][1],
                           parameters[8][2]);
  // gyroscope bias of 2nd imu frame (3)
  Eigen::Vector3d biasGyroJ(parameters[9][0], parameters[9][1],
                            parameters[9][2]);

  // calculate residuals
  Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);

  // correct result by newest biases

  Eigen::Vector3d deltaBiasAcc =
      biasAccI - imuIntegration.getLinearizedBiasAcc();
  Eigen::Vector3d deltaBiasGyro =
      biasGyroI - imuIntegration.getLinearizedBiasGyro();
  Eigen::Matrix3d jacobPosToBiasAcc =
      imuIntegration.getJacobian().block<3, 3>(0, 9);
  Eigen::Matrix3d jacobPosToBiasGyro =
      imuIntegration.getJacobian().block<3, 3>(0, 12);
  Eigen::Matrix3d jacobRotToBiasGyro =
      imuIntegration.getJacobian().block<3, 3>(3, 12);
  Eigen::Matrix3d jacobVelToBiasAcc =
      imuIntegration.getJacobian().block<3, 3>(6, 9);
  Eigen::Matrix3d jacobVelToBiasGyro =
      imuIntegration.getJacobian().block<3, 3>(6, 12);
  double totalTime = imuIntegration.getTotalTime();

  Eigen::Quaterniond correctedDeltaQ =
      imuIntegration.getDeltaQ() *
      MathUtility::deltaQ(jacobRotToBiasGyro * deltaBiasGyro);
  Eigen::Vector3d correctedDeltaV = imuIntegration.getDeltaV() +
                                    jacobVelToBiasAcc * deltaBiasAcc +
                                    jacobVelToBiasGyro * deltaBiasGyro;
  Eigen::Vector3d correctedDeltaP = imuIntegration.getDeltaP() +
                                    jacobPosToBiasAcc * deltaBiasAcc +
                                    jacobPosToBiasGyro * deltaBiasGyro;

  // position residuals
  residual.block<3, 1>(0, 0) =
      qIToW.inverse() * (0.5 * gInW * totalTime * totalTime + tJToW - tIToW -
                         velocityI * totalTime) -
      correctedDeltaP;
  // rotation residuals, slightly different with paper,
  // results are same, but jacobians are not
  residual.block<3, 1>(3, 0) =
      2 * (correctedDeltaQ.inverse() * (qIToW.inverse() * qJToW)).vec();
  // velocity residuals
  residual.block<3, 1>(6, 0) =
      qIToW.inverse() * (gInW * totalTime + velocityJ - velocityI) -
      correctedDeltaV;
  // accelerometer bias residual
  residual.block<3, 1>(9, 0) = biasAccJ - biasAccI;
  // gyroscope bias residual
  residual.block<3, 1>(12, 0) = biasGyroJ - biasGyroI;

  Eigen::Matrix<double, 15, 15> sqrtInfoMatrix =
      Eigen::LLT<Eigen::Matrix<double, 15, 15>>(
          imuIntegration.getCovariance().inverse())
          .matrixL()
          .transpose();
  residual = sqrtInfoMatrix * residual;
  // TODO
  // std::cout << "//////////" << std::endl << sqrtInfoMatrix << std::endl;

  if (jacobians == nullptr) return true;

  // compute jacobians

  // jacobian w.r.t i-th imu frame postion
  if (jacobians[0] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianPositionI(
        jacobians[0]);
    jacobianPositionI.setZero();
    // position error
    jacobianPositionI.block<3, 3>(0, 0) = -qIToW.inverse().toRotationMatrix();
    jacobianPositionI = sqrtInfoMatrix * jacobianPositionI;
  }

  // jacobian w.r.t i-th imu frame rotation
  if (jacobians[1] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobianRotationI(
        jacobians[1]);
    jacobianRotationI.setZero();
    // position error
    jacobianRotationI.block<3, 3>(0, 0) = MathUtility::skewSymmetric(
        qIToW.inverse() * (0.5 * gInW * totalTime * totalTime + tJToW - tIToW -
                           velocityI * totalTime));
    // rotation error
    jacobianRotationI.block<3, 3>(3, 0) =
        -(MathUtility::leftMultiplyQ(qJToW.inverse() * qIToW) *
          MathUtility::rightMultiplyQ(correctedDeltaQ))
             .bottomRightCorner<3, 3>();
    // velocity error
    jacobianRotationI.block<3, 3>(6, 0) = MathUtility::skewSymmetric(
        qIToW.inverse() * (gInW * totalTime + velocityJ - velocityI));
    jacobianRotationI = sqrtInfoMatrix * jacobianRotationI;
  }

  // jacobian w.r.t i-th imu speed
  if (jacobians[2] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianVelocityI(
        jacobians[2]);
    jacobianVelocityI.setZero();
    // position error
    jacobianVelocityI.block<3, 3>(0, 0) =
        -qIToW.inverse().toRotationMatrix() * totalTime;
    // velocity error
    jacobianVelocityI.block<3, 3>(6, 0) = -qIToW.inverse().toRotationMatrix();
    jacobianVelocityI = sqrtInfoMatrix * jacobianVelocityI;
  }

  // jacobian w.r.t i-th accelerometer bias
  if (jacobians[3] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianAccBiasI(
        jacobians[3]);
    jacobianAccBiasI.setZero();
    // position error
    jacobianAccBiasI.block<3, 3>(0, 0) = -jacobPosToBiasAcc;
    // velocity error
    jacobianAccBiasI.block<3, 3>(6, 0) = -jacobVelToBiasAcc;
    // accelerometer bias error
    jacobianAccBiasI.block<3, 3>(9, 0) = -Eigen::Matrix3d::Identity();
    jacobianAccBiasI = sqrtInfoMatrix * jacobianAccBiasI;
  }

  // jacobian w.r.t i-th gyroscope bias
  if (jacobians[4] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianGyroBiasI(
        jacobians[4]);
    jacobianGyroBiasI.setZero();
    // position error
    jacobianGyroBiasI.block<3, 3>(0, 0) = -jacobPosToBiasGyro;
    // rotation error
    jacobianGyroBiasI.block<3, 3>(3, 0) =
        -MathUtility::leftMultiplyQ(qJToW.inverse() * qIToW *
                                    imuIntegration.getDeltaQ())
             .bottomRightCorner<3, 3>() *
        jacobRotToBiasGyro;
    // velocity error
    jacobianGyroBiasI.block<3, 3>(6, 0) = -jacobVelToBiasGyro;
    // gyroscope bias error
    jacobianGyroBiasI.block<3, 3>(12, 0) = -Eigen::Matrix3d::Identity();
    jacobianGyroBiasI = sqrtInfoMatrix * jacobianGyroBiasI;
  }

  // jacobian w.r.t j-th imu frame postion
  if (jacobians[5] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianPositionJ(
        jacobians[5]);
    jacobianPositionJ.setZero();
    // position error
    jacobianPositionJ.block<3, 3>(0, 0) = qIToW.inverse().toRotationMatrix();
    jacobianPositionJ = sqrtInfoMatrix * jacobianPositionJ;
  }

  // jacobian w.r.t j-th imu frame rotation
  if (jacobians[6] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobianRotationJ(
        jacobians[6]);
    jacobianRotationJ.setZero();
    // rotation error
    jacobianRotationJ.block<3, 3>(3, 0) =
        MathUtility::leftMultiplyQ(correctedDeltaQ.inverse() * qIToW.inverse() *
                                   qJToW)
            .bottomRightCorner<3, 3>();
    jacobianRotationJ = sqrtInfoMatrix * jacobianRotationJ;
  }

  // jacobian w.r.t j-th imu speed
  if (jacobians[7] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianVelocityJ(
        jacobians[7]);
    jacobianVelocityJ.setZero();
    // velocity error
    jacobianVelocityJ.block<3, 3>(6, 0) = qIToW.inverse().toRotationMatrix();
    jacobianVelocityJ = sqrtInfoMatrix * jacobianVelocityJ;
  }

  // jacobian w.r.t j-th accelerometer bias
  if (jacobians[8] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianAccBiasJ(
        jacobians[8]);
    jacobianAccBiasJ.setZero();
    // accelerometer bias error
    jacobianAccBiasJ.block<3, 3>(9, 0) = Eigen::Matrix3d::Identity();
    jacobianAccBiasJ = sqrtInfoMatrix * jacobianAccBiasJ;
  }

  // jacobian w.r.t j-th gyroscope bias
  if (jacobians[9] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 15, 3, Eigen::RowMajor>> jacobianGyroBiasJ(
        jacobians[9]);
    jacobianGyroBiasJ.setZero();
    // accelerometer bias error
    jacobianGyroBiasJ.block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
    jacobianGyroBiasJ = sqrtInfoMatrix * jacobianGyroBiasJ;
  }

  return true;
}

}  // namespace Residuals