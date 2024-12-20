#include "RadarPointResidual.hpp"

#include "MathUtility.hpp"

namespace Residuals {

RadarPointResidual::RadarPointResidual(Eigen::Vector3d measurement,
                                       double sqrtInfoGainValue)
    : measuredPoint(measurement) {
  this->sqrtInfoGain << sqrtInfoGainValue, 0, 0, 0, sqrtInfoGainValue, 0, 0, 0,
      sqrtInfoGainValue;
}

RadarPointResidual::RadarPointResidual(Eigen::Vector3d measurement,
                                       double range, double azimuth,
                                       double elevation, double SigmaRange,
                                       double SigmaTheta, double SigmaPhi,
                                       double maxInfoGain, int observationCount,
                                       int pointNum)
    : measuredPoint(measurement),
      Range(range),
      Theta(azimuth),
      Phi(elevation),
      SigmaRange(SigmaRange),
      SigmaTheta(SigmaTheta),
      SigmaPhi(SigmaPhi),
      maxInfoGain(maxInfoGain),
      observationCount(observationCount),
      pointNum(pointNum) {
  // std::cout << "range: " << range << "\t" << "azimuth: " << azimuth << "\t"
  // << "elevation: " << elevation << std::endl;
  JRangeThetaPhi << cos(Theta) * cos(Phi), -Range * sin(Theta) * cos(Phi),
      -Range * cos(Theta) * sin(Phi), sin(Theta) * cos(Phi),
      Range * cos(Theta) * cos(Phi), -Range * sin(Theta) * sin(Phi), sin(Phi),
      0, -Range * cos(Phi);
  // std::cout << "JRangeThetaPhi: " << JRangeThetaPhi << std::endl;
  calculateSqrtInfoGain(Range, Theta, Phi);
  // std::cout << "sqrtInfoGain: " << sqrtInfoGain << std::endl;
}

void RadarPointResidual::calculateSqrtInfoGain(double range, double azimuth,
                                               double elevation) {
  Eigen::Matrix<double, 3, 3> Sigma;
  Sigma << SigmaRange * SigmaRange, 0, 0, 0, SigmaTheta * SigmaTheta, 0, 0, 0,
      SigmaPhi * SigmaPhi;
  Eigen::Matrix<double, 3, 3> Cov;
  Cov = JRangeThetaPhi * Sigma * JRangeThetaPhi.transpose();
  Eigen::Matrix<double, 3, 3> InfoGain = Cov.inverse();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> es(InfoGain);

  sqrtInfoGain = es.operatorSqrt();
  sqrtInfoGain = sqrtInfoGain * observationCount / pointNum;
  double scale = abs(sqrtInfoGain.maxCoeff() / maxInfoGain);
  scale = scale > abs(sqrtInfoGain.minCoeff() / maxInfoGain)
              ? scale
              : abs(sqrtInfoGain.minCoeff() / maxInfoGain);
  sqrtInfoGain = sqrtInfoGain / scale;

  if (es.info() != Eigen::Success || sqrtInfoGain.hasNaN()) {
    sqrtInfoGain << maxInfoGain, 0, 0, 0, maxInfoGain, 0, 0, 0, maxInfoGain;
  }

  // std::cout << "sqrtInfoGain: \n" << sqrtInfoGain << std::endl;
}

bool RadarPointResidual::Evaluate(double const *const *parameters,
                                  double *residuals, double **jacobians) const {
  // translation of i-th frame (3)
  Eigen::Vector3d tIToW(parameters[0][0], parameters[0][1], parameters[0][2]);
  // rotation of i-th frame (4)
  Eigen::Quaterniond qIToW(parameters[1][3], parameters[1][0], parameters[1][1],
                           parameters[1][2]);
  // translation of extrinsic (3)
  Eigen::Vector3d tRadarToImu(parameters[2][0], parameters[2][1],
                              parameters[2][2]);
  // rotation  of extrinsic (4)
  Eigen::Quaterniond qRadarToImu(parameters[3][3], parameters[3][0],
                                 parameters[3][1], parameters[3][2]);
  // point in world (3)
  Eigen::Vector3d pointState(parameters[4][0], parameters[4][1],
                             parameters[4][2]);

  Eigen::Map<Eigen::Vector3d> resultResidual(residuals);
  Eigen::Vector3d pointInIImu = qRadarToImu * measuredPoint + tRadarToImu;
  Eigen::Vector3d pointInWorld = qIToW * pointInIImu + tIToW;
  resultResidual = sqrtInfoGain * (pointState - pointInWorld);

  if (jacobians == nullptr) return true;

  // jacobians w.r.t translation of frame i
  if (jacobians[0] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobianPositionI(
        jacobians[0]);
    jacobianPositionI = -sqrtInfoGain * Eigen::Matrix3d::Identity();
  }

  // jacobians w.r.t rotation of frame i
  if (jacobians[1] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobianRotationI(
        jacobians[1]);
    jacobianRotationI.setZero();
    jacobianRotationI.block<3, 3>(0, 0) =
        sqrtInfoGain * qIToW.toRotationMatrix() *
        MathUtility::skewSymmetric(pointInIImu);
  }

  // jacobians w.r.t translation of extrinsic
  if (jacobians[2] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobianPositionEx(
        jacobians[2]);
    jacobianPositionEx = -sqrtInfoGain * qIToW.toRotationMatrix();
  }

  // jacobians w.r.t rotation of extrinsic
  if (jacobians[3] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobianRotationEx(
        jacobians[3]);
    jacobianRotationEx.setZero();
    jacobianRotationEx.block<3, 3>(0, 0) =
        sqrtInfoGain * qIToW.toRotationMatrix() *
        qRadarToImu.toRotationMatrix() *
        MathUtility::skewSymmetric(measuredPoint);
  }

  // jacobians w.r.t translation of point
  if (jacobians[4] != nullptr) {
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobianPositionEx(
        jacobians[4]);
    jacobianPositionEx = sqrtInfoGain * Eigen::Matrix3d::Identity();
  }

  return true;
}

}  // namespace Residuals