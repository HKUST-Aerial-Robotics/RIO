#pragma once

namespace State {
constexpr int SIZE_IMU_STATE = 9;

struct IMUState {
  // Vx,Vy,Vz
  double velocityParams[3];
  // Bax,Bay,Baz
  double biasAccParams[3];
  // Bgx,Bgy,Bgz
  double biasGyroParams[3];
};

}  // namespace State