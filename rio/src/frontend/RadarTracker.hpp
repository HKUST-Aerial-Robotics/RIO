#pragma once
#include <opencv2/core/hal/interface.h>

#include <cstdint>
#include <set>
#include <vector>

#include "RadarFrame.hpp"
#include "RadarPreprocessor.hpp"

namespace Frontend {
class RadarTracker {
  struct TrackingPointsInfo {
    uint32_t id;
    uint32_t count;
    Frame::RadarData data;
  };

 public:
  RadarTracker() = default;
  ~RadarTracker() = default;

  RadarTracker(RadarTracker &another) = delete;
  RadarTracker(RadarTracker &&another) = delete;
  RadarTracker &operator=(const RadarTracker &another) = delete;
  RadarTracker &operator=(RadarTracker &&another) = delete;

  void setMatchingThreshold(double maxRCSError, double maxDistanceError);

  void setMatchingParameters(double sigmaR, double sigmaTheta, double sigmaPhi,
                             double numSigma, bool useRCSFilter);

  void setPredictedVelocityThreshold(double maxVelocityError);

  Frame::RadarFrame trackPoints(std::vector<Frame::RadarData> points,
                                ros::Time time);

  void setPrediction(Eigen::Quaterniond previousRot,
                     Eigen::Vector3d previousVec, Eigen::Quaterniond currentRot,
                     Eigen::Vector3d currentVec, Eigen::Vector3d velInRadar);
  void removeTrackingPoints(const std::set<unsigned int> &idToRemove);

  Frame::RadarFrame mergeFrame(Eigen::Quaterniond previousRot,
                               Eigen::Vector3d previousVec,
                               Eigen::Quaterniond currentRot,
                               Eigen::Vector3d currentVec);

 private:
  void initialization(std::vector<Frame::RadarData> &points, ros::Time time);

  std::vector<uchar> searchCorrsponding(
      std::vector<Frame::RadarData> &previousPoints,
      std::vector<Frame::RadarData> &currentPoints,
      std::vector<Frame::RadarData> &detectedPoints,
      std::vector<Frame::RadarData> &undetectedPoints);

  std::vector<uchar> searchStaticPoints(
      std::vector<Frame::RadarData> &currentPoints, Eigen::Vector3d velocity);

  void trackAndDetectPoints(std::vector<Frame::RadarData> &points);

  double calculatePDF(Frame::RadarData &previousPoint,
                      Frame::RadarData &currentPoint);

  Frame::RadarFrame previousFrame;
  Frame::RadarFrame currentFrame;
  uint32_t featureId = 0;
  uint32_t frameId = 0;
  std::vector<TrackingPointsInfo> trackingPoints;
  std::vector<TrackingPointsInfo> previousTrackingPoints;

  bool hasPrediction = false;
  bool useRCSFilter = false;

  double rcsThreshold = 0;
  double distanceThreshold = 0;
  double velThreshold = 0;

  double SigmaR = 0;
  double SigmaTheta = 0;
  double SigmaPhi = 0;
  double pdfThreshold = 0;

  bool init = false;
  Eigen::Quaterniond prevRot{};
  Eigen::Vector3d prevVec{};
  Eigen::Quaterniond currRot{};
  Eigen::Vector3d currVec{};
  Eigen::Vector3d currVelInRadar{};
};
}  // namespace Frontend