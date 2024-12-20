#pragma once

#include <array>
#include <cstddef>
#include <deque>

#include "IMUState.hpp"
#include "R3State.hpp"
#include "RadarPointFeatureState.hpp"
#include "SE3State.hpp"

namespace State {
constexpr int MAX_WINDOWS_SIZE = 6;
constexpr int STATE_BUFFER_SIZE = 1;
constexpr int ARRAY_SIZE = MAX_WINDOWS_SIZE + STATE_BUFFER_SIZE;
constexpr int MAX_POINT_FEATURE_SIZE = 500 * MAX_WINDOWS_SIZE;

class StateManager {
 public:
  std::array<SE3State, ARRAY_SIZE> basicState{};
  int basicStateNum = 0;

  std::array<IMUState, ARRAY_SIZE> imuState{};
  int imuStateNum = 0;

  std::array<R3State, MAX_POINT_FEATURE_SIZE> pointState{};
  int pointNum = 0;

  SE3State exRadarToImu{};

  // template <typename T, std::size_t Nm>
  // void slideOutFront(std::array<T, Nm> array, int &number)
  // {
  //     for (int i = 0; i < number - 1; i++)
  //         array[i] = array[i + 1];
  //     number--;
  // }

  // template <typename T, std::size_t Nm>
  // void slideOutBack(std::array<T, Nm> array, int &number)
  // {
  //     number--;
  // }
};

}  // namespace State
