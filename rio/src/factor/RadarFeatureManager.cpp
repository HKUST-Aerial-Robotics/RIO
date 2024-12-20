#include "RadarFeatureManager.hpp"

#include <cstddef>

namespace Factor {
void RadarFeatureManager::pushBack(Frame::RadarFrame &frame) {
  int index = 0;
  RadarFrameRelation radarRelation;
  for (auto data : frame.matchedPoint) {
    radarRelation.featureId.emplace_back(data.first);
    while ((index < pointRelation.size()) &&
           pointRelation[index].featureId < data.first)
      index++;
    if (index >= pointRelation.size()) {
      PointFeatureRelation featureRelation;
      featureRelation.featureId = data.first;
      featureRelation.frameId.emplace_back(frame.id);
      featureRelation.measurement.emplace_back(data.second);
      featureRelation.init = false;
      featureRelation.gyroData.emplace_back(frame.gyroData);
      pointRelation.emplace_back(featureRelation);
      continue;
    }
    pointRelation[index].frameId.emplace_back(frame.id);
    pointRelation[index].measurement.emplace_back(data.second);
    pointRelation[index].gyroData.emplace_back(frame.gyroData);
  }
  for (auto data : frame.unmatchedPoint) {
    radarRelation.featureId.emplace_back(data.first);
    while ((index < pointRelation.size()) &&
           pointRelation[index].featureId < data.first)
      index++;
    if (index >= pointRelation.size()) {
      PointFeatureRelation featureRelation;
      featureRelation.featureId = data.first;
      featureRelation.frameId.emplace_back(frame.id);
      featureRelation.measurement.emplace_back(data.second);
      featureRelation.init = false;
      featureRelation.gyroData.emplace_back(frame.gyroData);
      pointRelation.emplace_back(featureRelation);
      continue;
    }
    pointRelation[index].frameId.emplace_back(frame.id);
    pointRelation[index].measurement.emplace_back(data.second);
    pointRelation[index].gyroData.emplace_back(frame.gyroData);
  }
  // push frame
  radarRelation.radarFrameId = frame.id;
  frameRelation.emplace_back(radarRelation);
}

void RadarFeatureManager::popFront() {
  int index = 0;
  // delete point relation
  for (int i = 0; i < frameRelation.front().featureId.size(); i++) {
    while (
        (index < pointRelation.size()) &&
        (pointRelation[index].featureId < frameRelation.front().featureId[i]))
      index++;
    if (index >= pointRelation.size()) break;
    // delete relation
    if (pointRelation[index].featureId == frameRelation.front().featureId[i]) {
      pointRelation[index].frameId.pop_front();
      pointRelation[index].measurement.pop_front();
      pointRelation[index].gyroData.pop_front();
    }
    // delete point
    if (pointRelation[index].frameId.empty())
      pointRelation.erase(pointRelation.begin() + index);
  }
  // delete frame
  frameRelation.pop_front();
}

void RadarFeatureManager::popBack() {
  int index = 0;
  // delete point relation
  for (int i = 0; i < frameRelation.back().featureId.size(); i++) {
    while ((index < pointRelation.size()) &&
           (pointRelation[index].featureId < frameRelation.back().featureId[i]))
      index++;
    if (index >= pointRelation.size()) break;
    // delete relation
    if (pointRelation[index].featureId == frameRelation.back().featureId[i]) {
      pointRelation[index].frameId.pop_back();
      pointRelation[index].measurement.pop_back();
      pointRelation[index].gyroData.pop_back();
    }
    // delete point
    if (pointRelation[index].frameId.empty())
      pointRelation.erase(pointRelation.begin() + index);
  }
  // delete frame
  frameRelation.pop_back();
}

void RadarFeatureManager::clear() {
  frameRelation.clear();
  pointRelation.clear();
}

size_t RadarFeatureManager::frameSize() const { return frameRelation.size(); }

size_t RadarFeatureManager::activeFeatureSize() const {
  size_t result = 0;
  for (auto point : pointRelation)
    if (point.frameId.size() > 1) result++;
  return result;
}

size_t RadarFeatureManager::featureSize() const { return pointRelation.size(); }

}  // namespace Factor