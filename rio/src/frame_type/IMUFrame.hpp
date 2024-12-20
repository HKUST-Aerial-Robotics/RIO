#pragma once

#include "eigen3/Eigen/Eigen"
#include "ros/time.h"

namespace Frame {
class IMUFrame {
 public:
  IMUFrame() = default;
  ~IMUFrame() = default;
  IMUFrame(IMUFrame &another) = default;
  IMUFrame(IMUFrame &&another) = default;
  IMUFrame &operator=(const IMUFrame &another) = default;
  IMUFrame &operator=(IMUFrame &&another) = default;

  Eigen::Vector3d accData;
  Eigen::Vector3d gyroData;
  uint32_t id = 0;
  ros::Time receivedTime{0, 0};
  // double receivedTime = 0;
};
}  // namespace Frame
