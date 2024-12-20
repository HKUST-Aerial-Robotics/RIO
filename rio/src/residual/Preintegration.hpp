#pragma once

#include <Eigen/Eigen>
#include <vector>

namespace Residuals {
class Preintegration {
  struct IMUData {
    Eigen::Vector3d accData;
    Eigen::Vector3d gyroData;
    double deltaTime;
  };

 public:
  Preintegration(Eigen::Vector3d accData, Eigen::Vector3d gyroData,
                 Eigen::Vector3d biasAcc, Eigen::Vector3d biasGyro,
                 double accNoise, double gyroNoise, double accBiasNoise,
                 double gyroBiasNoise);
  ~Preintegration() = default;
  Preintegration() = delete;
  Preintegration(Preintegration &another) = default;
  Preintegration(Preintegration &&another) = default;
  Preintegration &operator=(const Preintegration &another) = default;
  Preintegration &operator=(Preintegration &&another) = default;

  void clearData();
  void propagate(double deltaTime, Eigen::Vector3d accData,
                 Eigen::Vector3d gyroData);
  void repropagate(Eigen::Vector3d biasAcc, Eigen::Vector3d biasGyro);

  Eigen::Vector3d getDeltaP() const;
  Eigen::Quaterniond getDeltaQ() const;
  Eigen::Vector3d getDeltaV() const;
  Eigen::Matrix<double, 15, 15> getJacobian() const;
  Eigen::Matrix<double, 15, 15> getCovariance() const;
  Eigen::Vector3d getLinearizedBiasAcc() const;
  Eigen::Vector3d getLinearizedBiasGyro() const;
  double getTotalTime() const;

 private:
  void integrate(double deltaTime, Eigen::Vector3d startAcc,
                 Eigen::Vector3d startGyro, Eigen::Vector3d endAcc,
                 Eigen::Vector3d endGyro, Eigen::Vector3d biasAcc,
                 Eigen::Vector3d biasGyro, Eigen::Vector3d &deltaPos,
                 Eigen::Quaterniond &deltaRot, Eigen::Vector3d &deltaVel);

  Eigen::Vector3d deltaP;
  Eigen::Quaterniond deltaQ;
  Eigen::Vector3d deltaV;

  std::vector<IMUData> imuBuffer;
  Eigen::Vector3d linearizedBiasAcc;
  Eigen::Vector3d linearizedBiasGyro;

  Eigen::Matrix<double, 18, 18> noise;
  Eigen::Matrix<double, 15, 15> jacobian;
  Eigen::Matrix<double, 15, 15> covariance;

  double totalTime = 0;
};
}  // namespace Residuals