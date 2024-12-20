#pragma once

namespace State {

constexpr int SIZE_RADAR_POINT_FEATURE_STATE = 2;

struct RadarPointFeatureState {
  double azimuth[1];
  double elevation[1];
};

}  // namespace State