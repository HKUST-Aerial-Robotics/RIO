#pragma once

namespace State {
constexpr int SIZE_SE3_STATE = 7;

struct SE3State {
  // Px,Py,Pz
  double positionParams[3];
  // Qx,Qy,Qz,Qw
  double rotationParams[4];
};

}  // namespace State