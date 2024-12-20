#pragma once

#include <cmath>

#include "Eigen/Eigen"

namespace MathUtility {
static Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &rotationVector) {
  Eigen::Matrix3d result;
  // clang-format off
    result <<   0.0,                -rotationVector.z(), rotationVector.y(), 
                rotationVector.z(), 0.0,                -rotationVector.x(), 
                -rotationVector.y(),rotationVector.x(), 0.0;
  // clang-format on
  return result;
}

static Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta) {
  Eigen::Quaterniond result;
  Eigen::Vector3d halfTheta = theta / 2.0;
  result.w() = 1.0;
  result.x() = halfTheta.x();
  result.y() = halfTheta.y();
  result.z() = halfTheta.z();
  return result;
}

static Eigen::Vector3d vectorQ(const Eigen::Quaterniond &quat) {
  Eigen::Vector3d result;
  double norm = std::hypot(quat.x(), quat.y(), quat.z());
  if (std::fpclassify(norm) == FP_ZERO) return result.setZero();
  double theta = std::atan2(norm, quat.w());
  result.x() = theta * quat.x() / norm;
  result.y() = theta * quat.y() / norm;
  result.z() = theta * quat.z() / norm;
  return result;
}

static Eigen::Matrix4d leftMultiplyQ(const Eigen::Quaterniond &quat) {
  Eigen::Matrix4d result;
  result(0, 0) = quat.w();
  result.block<1, 3>(0, 1) = -quat.vec().transpose();
  result.block<3, 1>(1, 0) = quat.vec();
  result.block<3, 3>(1, 1) =
      quat.w() * Eigen::Matrix3d::Identity() + skewSymmetric(quat.vec());
  return result;
}

static Eigen::Matrix4d rightMultiplyQ(const Eigen::Quaterniond &quat) {
  Eigen::Matrix4d result;
  result(0, 0) = quat.w();
  result.block<1, 3>(0, 1) = -quat.vec().transpose();
  result.block<3, 1>(1, 0) = quat.vec();
  result.block<3, 3>(1, 1) =
      quat.w() * Eigen::Matrix3d::Identity() - skewSymmetric(quat.vec());
  return result;
}

}  // namespace MathUtility