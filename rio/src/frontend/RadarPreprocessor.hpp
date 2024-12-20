#pragma once

#include <memory>
#include <vector>

#include "RadarFrame.hpp"

namespace Frontend {
class RadarPreprocessor {
 public:
  struct FOVParams {
    float minElevation;
    float maxElevation;
    float minAzimuth;
    float maxAzimuth;
    float minDistance;
    float maxDistance;
  };

  struct VelParams {
    float minVelocity;
    float maxVelocity;
  };

  struct SpatialFilterParams {
    int number;
    float radius;
    float sigma;
  };

  RadarPreprocessor() = default;
  ~RadarPreprocessor() = default;

  RadarPreprocessor(RadarPreprocessor &another) = delete;
  RadarPreprocessor(RadarPreprocessor &&another) = delete;
  RadarPreprocessor &operator=(const RadarPreprocessor &another) = delete;
  RadarPreprocessor &operator=(RadarPreprocessor &&another) = delete;

  void process(std::vector<Frame::RadarData> &frame) const;

  void addFOVParams(FOVParams param);
  void setVelParams(VelParams param);
  void setDistanceParams(double distance);
  void setSpatialParams(SpatialFilterParams param);

 private:
  void fovFilter(std::vector<Frame::RadarData> &frame) const;
  void velocityFilter(std::vector<Frame::RadarData> &frame) const;
  void distanceFilter(std::vector<Frame::RadarData> &frame) const;
  void spatialFilter(std::vector<Frame::RadarData> &frame) const;

  std::vector<FOVParams> fovParams{};
  VelParams velParams{};
  double maxDistance = 0;
  SpatialFilterParams spatialParams{};

  bool enFovFilter = false;
  bool enVelFilter = false;
  bool enDistFilter = false;
  bool enSpatFilter = false;
};
}  // namespace Frontend