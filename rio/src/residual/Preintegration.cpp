#include "Preintegration.hpp"

#include <MathUtility.hpp>
#include <utility>

namespace Residuals {
Preintegration::Preintegration(Eigen::Vector3d accData,
                               Eigen::Vector3d gyroData,
                               Eigen::Vector3d biasAcc,
                               Eigen::Vector3d biasGyro, double accNoise,
                               double gyroNoise, double accBiasNoise,
                               double gyroBiasNoise)
    : linearizedBiasAcc(std::move(biasAcc)),
      linearizedBiasGyro(std::move(biasGyro)) {
  deltaP.setZero();
  deltaV.setZero();
  deltaQ.setIdentity();
  jacobian.setIdentity();
  covariance.setZero();
  imuBuffer.emplace_back(IMUData{accData, gyroData, 0});
  noise.setZero();
  noise.block<3, 3>(0, 0) = (accNoise * accNoise) * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(3, 3) = gyroNoise * gyroNoise * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(6, 6) = (accNoise * accNoise) * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(9, 9) =
      (gyroNoise * gyroNoise) * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(12, 12) =
      (accBiasNoise * accBiasNoise) * Eigen::Matrix3d::Identity();
  noise.block<3, 3>(15, 15) =
      (gyroBiasNoise * gyroBiasNoise) * Eigen::Matrix3d::Identity();
}

void Preintegration::clearData() {
  deltaP.setZero();
  deltaV.setZero();
  deltaQ.setIdentity();
  imuBuffer.clear();
  totalTime = 0;
}

void Preintegration::propagate(double deltaTime, Eigen::Vector3d accData,
                               Eigen::Vector3d gyroData) {
  imuBuffer.emplace_back(IMUData{accData, gyroData, deltaTime});
  integrate(imuBuffer.back().deltaTime, (imuBuffer.end() - 2)->accData,
            (imuBuffer.end() - 2)->gyroData, imuBuffer.back().accData,
            imuBuffer.back().gyroData, linearizedBiasAcc, linearizedBiasGyro,
            deltaP, deltaQ, deltaV);
  totalTime += deltaTime;
}

void Preintegration::repropagate(Eigen::Vector3d biasAcc,
                                 Eigen::Vector3d biasGyro) {
  deltaP.setZero();
  deltaV.setZero();
  deltaQ.setIdentity();
  linearizedBiasAcc = biasAcc;
  linearizedBiasGyro = biasGyro;
  jacobian.setIdentity();
  covariance.setZero();
  for (auto iter = imuBuffer.begin() + 1; iter != imuBuffer.end(); iter++)
    integrate(iter->deltaTime, (iter - 1)->accData, (iter - 1)->gyroData,
              iter->accData, iter->gyroData, linearizedBiasAcc,
              linearizedBiasGyro, deltaP, deltaQ, deltaV);
}

Eigen::Vector3d Preintegration::getDeltaP() const { return deltaP; }

Eigen::Quaterniond Preintegration::getDeltaQ() const { return deltaQ; }

Eigen::Vector3d Preintegration::getDeltaV() const { return deltaV; }

Eigen::Matrix<double, 15, 15> Preintegration::getJacobian() const {
  return jacobian;
};

Eigen::Matrix<double, 15, 15> Preintegration::getCovariance() const {
  return covariance;
};

double Preintegration::getTotalTime() const { return totalTime; }

Eigen::Vector3d Preintegration::getLinearizedBiasAcc() const {
  return linearizedBiasAcc;
}
Eigen::Vector3d Preintegration::getLinearizedBiasGyro() const {
  return linearizedBiasGyro;
}

void Preintegration::integrate(
    double deltaTime, Eigen::Vector3d startAcc, Eigen::Vector3d startGyro,
    Eigen::Vector3d endAcc, Eigen::Vector3d endGyro, Eigen::Vector3d biasAcc,
    Eigen::Vector3d biasGyro, Eigen::Vector3d &deltaPos,
    Eigen::Quaterniond &deltaRot, Eigen::Vector3d &deltaVel) {
  // mid-point integration
  Eigen::Vector3d resultDeltaPos;
  Eigen::Quaterniond resultDeltaRot;
  Eigen::Vector3d resultDeltaVel;

  // rotation result
  Eigen::Vector3d unbiasedGyro = 0.5 * (startGyro + endGyro) - biasGyro;
  resultDeltaRot =
      deltaRot * Eigen::Quaterniond(1, unbiasedGyro(0) * deltaTime / 2,
                                    unbiasedGyro(1) * deltaTime / 2,
                                    unbiasedGyro(2) * deltaTime / 2);

  // Why not normalize here?
  // resultDeltaRot.normalize();

  // position and velocity result
  Eigen::Vector3d unbiasedStartAcc = startAcc - biasAcc;
  Eigen::Vector3d unbiasedEndAcc = endAcc - biasAcc;
  Eigen::Vector3d unbiasedAcc =
      0.5 * (deltaRot * unbiasedStartAcc + resultDeltaRot * unbiasedEndAcc);
  resultDeltaPos = deltaPos + deltaVel * deltaTime +
                   0.5 * unbiasedAcc * deltaTime * deltaTime;
  resultDeltaVel = deltaVel + unbiasedAcc * deltaTime;

  // update jacobian
  // process model
  Eigen::MatrixXd processModel = Eigen::MatrixXd::Zero(15, 15);

  // position
  processModel.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  processModel.block<3, 3>(0, 3) =
      -0.25 * deltaRot.toRotationMatrix() *
          MathUtility::skewSymmetric(unbiasedStartAcc) * deltaTime * deltaTime +
      -0.25 * resultDeltaRot.toRotationMatrix() *
          MathUtility::skewSymmetric(unbiasedEndAcc) *
          (Eigen::Matrix3d::Identity() -
           MathUtility::skewSymmetric(unbiasedGyro) * deltaTime) *
          deltaTime * deltaTime;
  processModel.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity() * deltaTime;
  processModel.block<3, 3>(0, 9) =
      -0.25 *
      (deltaRot.toRotationMatrix() + resultDeltaRot.toRotationMatrix()) *
      deltaTime * deltaTime;
  processModel.block<3, 3>(0, 12) = 0.25 * resultDeltaRot.toRotationMatrix() *
                                    MathUtility::skewSymmetric(unbiasedEndAcc) *
                                    deltaTime * deltaTime * deltaTime;

  // rotation
  processModel.block<3, 3>(3, 3) =
      Eigen::Matrix3d::Identity() -
      MathUtility::skewSymmetric(unbiasedGyro) * deltaTime;
  processModel.block<3, 3>(3, 12) = -deltaTime * Eigen::Matrix3d::Identity();

  // velocity
  processModel.block<3, 3>(6, 3) =
      -0.5 * deltaRot.toRotationMatrix() *
          MathUtility::skewSymmetric(unbiasedStartAcc) * deltaTime +
      -0.5 * resultDeltaRot.toRotationMatrix() *
          MathUtility::skewSymmetric(unbiasedEndAcc) *
          (Eigen::Matrix3d::Identity() -
           MathUtility::skewSymmetric(unbiasedGyro) * deltaTime) *
          deltaTime;
  processModel.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
  processModel.block<3, 3>(6, 9) =
      -0.5 * (deltaRot.toRotationMatrix() + resultDeltaRot.toRotationMatrix()) *
      deltaTime;
  processModel.block<3, 3>(6, 12) = 0.5 * resultDeltaRot.toRotationMatrix() *
                                    MathUtility::skewSymmetric(unbiasedEndAcc) *
                                    deltaTime * deltaTime;

  // accelerometer bias
  processModel.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity();

  // gyroscope bias
  processModel.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity();

  // noise model
  Eigen::MatrixXd noiseModel = Eigen::MatrixXd::Zero(15, 18);

  // position
  noiseModel.block<3, 3>(0, 0) =
      0.25 * deltaRot.toRotationMatrix() * deltaTime * deltaTime;
  noiseModel.block<3, 3>(0, 3) = -0.125 * resultDeltaRot.toRotationMatrix() *
                                 MathUtility::skewSymmetric(unbiasedEndAcc) *
                                 deltaTime * deltaTime * deltaTime;
  noiseModel.block<3, 3>(0, 6) =
      0.25 * resultDeltaRot.toRotationMatrix() * deltaTime * deltaTime;
  noiseModel.block<3, 3>(0, 9) = -0.125 * resultDeltaRot.toRotationMatrix() *
                                 MathUtility::skewSymmetric(unbiasedEndAcc) *
                                 deltaTime * deltaTime * deltaTime;

  // rotation
  noiseModel.block<3, 3>(3, 3) = 0.5 * Eigen::Matrix3d::Identity() * deltaTime;
  noiseModel.block<3, 3>(3, 9) = 0.5 * Eigen::Matrix3d::Identity() * deltaTime;

  // velocity
  noiseModel.block<3, 3>(6, 0) = 0.5 * deltaRot.toRotationMatrix() * deltaTime;
  noiseModel.block<3, 3>(6, 3) = -0.25 * resultDeltaRot.toRotationMatrix() *
                                 MathUtility::skewSymmetric(unbiasedEndAcc) *
                                 deltaTime * deltaTime;
  noiseModel.block<3, 3>(6, 6) =
      0.5 * resultDeltaRot.toRotationMatrix() * deltaTime;
  noiseModel.block<3, 3>(6, 9) = -0.25 * resultDeltaRot.toRotationMatrix() *
                                 MathUtility::skewSymmetric(unbiasedEndAcc) *
                                 deltaTime * deltaTime;

  // accelerometer bias
  noiseModel.block<3, 3>(9, 12) = Eigen::Matrix3d::Identity() * deltaTime;

  // gyroscope bias
  noiseModel.block<3, 3>(12, 15) = Eigen::Matrix3d::Identity() * deltaTime;

  // update result
  jacobian = processModel * jacobian;
  covariance = processModel * covariance * processModel.transpose() +
               noiseModel * noise * noiseModel.transpose();

  // due to approximation the result quaternion may not be legal
  resultDeltaRot.normalize();

  deltaPos = resultDeltaPos;
  deltaRot = resultDeltaRot;
  deltaVel = resultDeltaVel;
}

}  // namespace Residuals