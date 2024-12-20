#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>

#include "RadarFrame.hpp"

namespace Factor {

// it presents a graph
class RadarFeatureManager {
 public:
  struct RadarFrameRelation;
  struct PointFeatureRelation;

  struct RadarFrameRelation {
    uint32_t radarFrameId;
    std::deque<uint32_t> featureId;
  };

  struct PointFeatureRelation {
    uint32_t featureId;
    std::deque<uint32_t> frameId;
    std::deque<Eigen::Vector3d> gyroData;
    Eigen::Vector3d pointState;
    bool init;
    std::deque<Frame::RadarData> measurement;
  };

  std::deque<RadarFrameRelation> frameRelation;
  std::deque<PointFeatureRelation> pointRelation;

  void pushBack(Frame::RadarFrame &frame);
  void popFront();
  void popBack();
  void mergeBack(Eigen::Quaterniond previousRot, Eigen::Vector3d previousVec,
                 Eigen::Quaterniond currentRot, Eigen::Vector3d currentVec);
  void clear();
  size_t frameSize() const;
  size_t activeFeatureSize() const;
  size_t featureSize() const;
};

}  // namespace Factor