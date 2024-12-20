#pragma once

namespace State {
constexpr int SIZE_R3_STATE = 3;

struct R3State {
  // Px,Py,Pz
  double positionParams[3];
};

}  // namespace State