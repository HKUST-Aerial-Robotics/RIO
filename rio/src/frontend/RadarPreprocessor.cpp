#include "RadarPreprocessor.hpp"

#include <opencv2/core/hal/interface.h>

#include <cmath>
#include <vector>

#include "RadarFrame.hpp"
#include "Utility.hpp"

namespace Frontend {
void RadarPreprocessor::process(std::vector<Frame::RadarData> &frame) const {
  if (enFovFilter) fovFilter(frame);
  if (enVelFilter) velocityFilter(frame);
  if (enDistFilter) distanceFilter(frame);
  if (enSpatFilter) spatialFilter(frame);
};

void RadarPreprocessor::addFOVParams(FOVParams param) {
  fovParams.emplace_back(param);
  enFovFilter = true;
};

void RadarPreprocessor::setVelParams(VelParams param) {
  velParams = param;
  enVelFilter = true;
};

void RadarPreprocessor::setDistanceParams(double distance) {
  maxDistance = distance;
  enDistFilter = true;
}

void RadarPreprocessor::setSpatialParams(SpatialFilterParams param) {
  spatialParams = param;
  enSpatFilter = true;
}

void RadarPreprocessor::fovFilter(std::vector<Frame::RadarData> &frame) const {
  if (frame.empty()) return;
  std::vector<uchar> status(frame.size(), 0);
  for (auto param : fovParams) {
    for (int index = 0; index < frame.size(); index++) {
      auto data = frame.at(index);
      if ((data.azimuth < param.maxAzimuth) &&
          (data.azimuth > param.minAzimuth) &&
          (data.elevation < param.maxElevation) &&
          (data.elevation > param.minElevation) &&
          (data.range < param.maxDistance) && (data.range > param.minDistance))
        status.at(index) = 1;
    }
  }
  reduceVector(frame, status);
}

void RadarPreprocessor::velocityFilter(
    std::vector<Frame::RadarData> &frame) const {
  if (frame.empty()) return;
  std::vector<uchar> status;
  for (auto data : frame)
    if ((data.doppler < velParams.maxVelocity) &&
        (data.doppler > velParams.minVelocity))
      status.emplace_back(1);
    else
      status.emplace_back(0);
  reduceVector(frame, status);
}

void RadarPreprocessor::distanceFilter(
    std::vector<Frame::RadarData> &frame) const {
  if (frame.size() <= 1) return;
  // TODO use KD-tree
  std::vector<uchar> status(frame.size(), 0);
  for (int i = 0; i < frame.size() - 1; i++)
    for (int j = i + 1; j < frame.size(); j++) {
      double deltaX = frame.at(i).x - frame.at(j).x;
      double deltaY = frame.at(i).y - frame.at(j).y;
      double deltaZ = frame.at(i).z - frame.at(j).z;
      double dis = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
      if (dis < maxDistance) {
        status.at(i) = 1;
        status.at(j) = 1;
      }
    }
  reduceVector(frame, status);
}

void RadarPreprocessor::spatialFilter(
    std::vector<Frame::RadarData> &frame) const {
  // TODO use KD-tree
  if (frame.size() <= 1) return;
  std::vector<std::vector<double>> distanceArray;
  for (int i = 0; i < frame.size(); i++) {
    distanceArray.emplace_back(std::vector<double>());
    for (int j = 0; j < frame.size(); j++) {
      double deltaX = frame.at(i).x - frame.at(j).x;
      double deltaY = frame.at(i).y - frame.at(j).y;
      double deltaZ = frame.at(i).z - frame.at(j).z;
      double dis = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
      if (dis < spatialParams.radius) distanceArray.back().emplace_back(dis);
    }
  }
  std::vector<uchar> status;
  for (auto &disVec : distanceArray) {
    if (disVec.size() > spatialParams.number)
      status.emplace_back(1);
    else
      status.emplace_back(0);
  }
  reduceVector(frame, status);
  reduceVector(distanceArray, status);

  if (frame.empty()) return;

  status.clear();
  for (auto &disVec : distanceArray) {
    double mean = 0;
    double std = 0;
    for (auto dis : disVec) mean += dis;
    mean /= (disVec.size() - 1);  // NOLINT : unsigned long to double
    for (auto dis : disVec) std += (dis - mean) * (dis - mean);
    std = std::sqrt(std /
                    (disVec.size() - 1));  // NOLINT : unsigned long to double
    if (std < spatialParams.sigma)
      status.emplace_back(1);
    else
      status.emplace_back(0);
  }
  reduceVector(frame, status);
}

}  // namespace Frontend