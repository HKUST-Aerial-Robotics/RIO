#include "RadarTracker.hpp"

#include <cmath>
#include <fstream>
#include <vector>

#include "RadarFrame.hpp"
#include "Utility.hpp"
#include "opencv2/core/hal/interface.h"
#include "ros/time.h"

static inline double pdf(double x, double mu, double sigma) {
  return exp(-0.5 * pow((x - mu) / sigma, 2)) / (sigma * sqrt(2 * M_PI));
}

namespace Frontend {
void RadarTracker::setMatchingThreshold(double maxRCSError,
                                        double maxDistanceError) {
  rcsThreshold = maxRCSError;
  distanceThreshold = maxDistanceError;
}

void RadarTracker::setPredictedVelocityThreshold(double maxVelocityError) {
  velThreshold = maxVelocityError;
}

void RadarTracker::setMatchingParameters(double sigmaR, double sigmaTheta,
                                         double sigmaPhi, double numSigma,
                                         bool useRCSFilter) {
  useRCSFilter = useRCSFilter;
  SigmaR = sigmaR;
  SigmaTheta = sigmaTheta;
  SigmaPhi = sigmaPhi;
  if (numSigma < 10e-6) {
    pdfThreshold = 0;
  } else {
    pdfThreshold = pdf(numSigma * sigmaR, 0, SigmaR) *
                   pdf(numSigma * sigmaTheta, 0, SigmaTheta) *
                   pdf(numSigma * sigmaPhi, 0, SigmaPhi);
  }
}

Frame::RadarFrame RadarTracker::trackPoints(
    std::vector<Frame::RadarData> points, ros::Time time) {
  previousFrame = currentFrame;
  if (!init) {
    init = true;
    initialization(points, time);
    return currentFrame;
  }
  // detected tracked point and new corresponding
  currentFrame.data = points;
  frameId++;
  currentFrame.id = frameId;
  currentFrame.receivedTime = time;
  currentFrame.matchedPoint.clear();
  currentFrame.unmatchedPoint.clear();
  currentFrame.staticPoint.clear();
  trackAndDetectPoints(points);
  // previousFrame = currentFrame;
  return currentFrame;
}

void RadarTracker::setPrediction(Eigen::Quaterniond previousRot,
                                 Eigen::Vector3d previousVec,
                                 Eigen::Quaterniond currentRot,
                                 Eigen::Vector3d currentVec,
                                 Eigen::Vector3d velInRadar) {
  hasPrediction = true;
  prevRot = previousRot;
  prevVec = previousVec;
  currRot = currentRot;
  currVec = currentVec;
  currVelInRadar = velInRadar;
}

void RadarTracker::removeTrackingPoints(
    const std::set<unsigned int> &idToRemove) {}

Frame::RadarFrame RadarTracker::mergeFrame(Eigen::Quaterniond previousRot,
                                           Eigen::Vector3d previousVec,
                                           Eigen::Quaterniond currentRot,
                                           Eigen::Vector3d currentVec) {
  // transform unmatched point back
  for (auto pts : currentFrame.unmatchedPoint) {
    auto transformed =
        Eigen::Vector3d{pts.second.x, pts.second.y, pts.second.z};
    transformed = previousRot.inverse() *
                  ((currentRot * transformed + currentVec) - previousVec);
    pts.second.x = transformed.x();
    pts.second.y = transformed.y();
    pts.second.z = transformed.z();
    Frame::RadarData::xyzToAngles(pts.second);
    previousTrackingPoints.emplace_back(
        TrackingPointsInfo{pts.first, 1, pts.second});
    previousFrame.unmatchedPoint[pts.first] = pts.second;
  }
  trackingPoints = previousTrackingPoints;
  currentFrame = previousFrame;
  return currentFrame;
}

void RadarTracker::initialization(std::vector<Frame::RadarData> &points,
                                  ros::Time time) {
  currentFrame.data = points;
  frameId++;
  currentFrame.id = frameId;
  currentFrame.matchedPoint.clear();
  currentFrame.unmatchedPoint.clear();
  currentFrame.staticPoint.clear();
  currentFrame.receivedTime = time;
  for (auto iter : points) {
    featureId++;
    trackingPoints.emplace_back(TrackingPointsInfo{featureId, 1, iter});
    currentFrame.unmatchedPoint[featureId] = iter;
  }
}

std::vector<uchar> RadarTracker::searchCorrsponding(
    std::vector<Frame::RadarData> &previousPoints,
    std::vector<Frame::RadarData> &currentPoints,
    std::vector<Frame::RadarData> &detectedPoints,
    std::vector<Frame::RadarData> &undetectedPoints) {
  // tracking points
  std::vector<uchar> result;
  detectedPoints.clear();
  undetectedPoints.clear();
  std::vector<uchar> undetectedStatus(currentPoints.size(), 1);
  std::vector<std::pair<int, double>> matchScore(currentPoints.size(), {-1, 0});

  for (int j = 0; j < previousPoints.size(); j++) {
    auto previousData = previousPoints[j];
    double minDistance = 1000;
    double maxPDF = 0;
    int minIndex = 0;
    Frame::RadarData minData{};
    // pdfThreshold = 0;
    if (pdfThreshold < 10e-6) {
      // Distance Based
      for (int i = 0; i < currentPoints.size(); i++) {
        auto currentData = currentPoints[i];
        // Radar Cross Section difference
        if (!useRCSFilter ||
            abs(previousData.rcs - currentData.rcs) <= rcsThreshold) {
          auto pointA =
              Eigen::Vector3d{previousData.x, previousData.y, previousData.z};
          auto pointB =
              Eigen::Vector3d{currentData.x, currentData.y, currentData.z};
          auto distance = (pointA - pointB).norm();
          if (distance < minDistance) {
            minDistance = distance;
            minData = currentData;
            minIndex = i;
          }
        }
      }
      // Distance Threshold
      if (minDistance < distanceThreshold) {
        result.emplace_back(1);
        undetectedStatus.at(minIndex) = 0;
        detectedPoints.emplace_back(minData);
      } else
        result.emplace_back(0);
    } else {
      // Convariance Based
      for (int i = 0; i < currentPoints.size(); i++) {
        auto currentData = currentPoints[i];
        Frame::RadarData::xyzToAngles(currentData);

        if (!useRCSFilter ||
            abs(previousData.rcs - currentData.rcs) <= rcsThreshold) {
          double PDF = calculatePDF(currentData, previousData);
          if (PDF > maxPDF) {
            maxPDF = PDF;
            minData = currentData;
            minIndex = i;
          }
        }
      }
      // PDF Threshold
      if (maxPDF > pdfThreshold) {
        result.emplace_back(1);
        undetectedStatus.at(minIndex) = 0;
        detectedPoints.emplace_back(minData);
      } else
        result.emplace_back(0);
    }
  }

  for (int i = 0; i < matchScore.size(); i++) {
    if (matchScore[i].first != -1) {
      detectedPoints.emplace_back(currentPoints[i]);
      Eigen::Vector3d PointA{previousPoints[matchScore[i].first].x,
                             previousPoints[matchScore[i].first].y,
                             previousPoints[matchScore[i].first].z};
      Eigen::Vector3d PointB{currentPoints[i].x, currentPoints[i].y,
                             currentPoints[i].z};
      double distance = (PointA - PointB).norm();
      // std::cout << "Distance: " << distance << std::endl;
    }
  }

  for (int i = 0; i < undetectedStatus.size(); i++)
    if (undetectedStatus.at(i) == 1)
      undetectedPoints.emplace_back(currentPoints[i]);
  return result;
}

std::vector<uchar> RadarTracker::searchStaticPoints(
    std::vector<Frame::RadarData> &currentPoints, Eigen::Vector3d velocity) {
  std::vector<uchar> result;
  for (auto data : currentPoints) {
    // sqrtInfoGain not used
    double sqrtInfoGain;
    Eigen::Matrix<double, 2, 2> Sigma;
    Eigen::Matrix<double, 3, 2> JThetaPhi;
    double Theta = data.azimuth;
    double Phi = data.elevation;
    JThetaPhi << -sin(Theta) * cos(Phi), -cos(Theta) * sin(Phi),
        cos(Theta) * cos(Phi), -sin(Theta) * sin(Phi), 0, cos(Phi);
    Sigma << SigmaTheta * SigmaTheta, 0, 0, SigmaPhi * SigmaPhi;
    sqrtInfoGain = sqrt(1.0 / (velocity.transpose() * JThetaPhi * Sigma *
                               JThetaPhi.transpose() * velocity));

    Eigen::Vector3d dir{data.x, data.y, data.z};
    dir.normalize();
    double velDrift = abs(data.doppler - dir.dot(velocity));

    if (velDrift < velThreshold) {
      result.emplace_back(1);
    } else {
      result.emplace_back(0);
    }
  }
  return result;
}

void RadarTracker::trackAndDetectPoints(std::vector<Frame::RadarData> &points) {
  std::vector<Frame::RadarData> detectedPoints;
  std::vector<Frame::RadarData> undetectedPoints;
  std::vector<Frame::RadarData> previousPoints;
  std::vector<uchar> status;
  if (!hasPrediction) {
    // tracking with previous
    for (auto pts : trackingPoints) previousPoints.emplace_back(pts.data);
    status = searchCorrsponding(previousPoints, points, detectedPoints,
                                undetectedPoints);
  } else {
    // track with prediction
    // since it is more accurate than position measurement.
    // velocity prediction, we process it separately
    status = searchStaticPoints(points, currVelInRadar);
    currentFrame.staticPoint = points;
    reduceVector(currentFrame.staticPoint, status);

    for (auto pts : trackingPoints) {
      // calculate the prediction points
      Eigen::Vector3d trackingPts{pts.data.x, pts.data.y, pts.data.z};
      trackingPts =
          currRot.inverse() * ((prevRot * trackingPts + prevVec) - currVec);
      pts.data.x = trackingPts.x();
      pts.data.y = trackingPts.y();
      pts.data.z = trackingPts.z();
      Frame::RadarData::xyzToAngles(pts.data);
      previousPoints.emplace_back(pts.data);
    }
    status = searchCorrsponding(previousPoints, currentFrame.staticPoint,
                                detectedPoints, undetectedPoints);
#ifdef DEBUG
    std::cout << "currentFrame.staticPoint.size(): "
              << currentFrame.staticPoint.size() << std::endl;
    std::cout << "detectedPoints.size(): " << detectedPoints.size()
              << std::endl;
    std::cout << "undetectedPoints.size(): " << undetectedPoints.size()
              << std::endl;
#endif
    // prediction check failed, fallback
    int successCnt = 0;
    for (auto i = 0; i < status.size(); i++)
      if (status[i] != 0) successCnt++;
    previousPoints.clear();
    if (successCnt <= 3) {
      for (auto pts : trackingPoints) {
        // calculate the prediction points
        Eigen::Vector3d trackingPts{pts.data.x, pts.data.y, pts.data.z};
        trackingPts =
            currRot.inverse() * ((prevRot * trackingPts + prevVec) - currVec);
        pts.data.x = trackingPts.x();
        pts.data.y = trackingPts.y();
        pts.data.z = trackingPts.z();
        Frame::RadarData::xyzToAngles(pts.data);
        previousPoints.emplace_back(pts.data);
      }
      status = searchCorrsponding(previousPoints, points, detectedPoints,
                                  undetectedPoints);
    }
  }
  previousTrackingPoints = trackingPoints;
  reduceVector(trackingPoints, status);

  // save result
  for (auto i = 0; i < detectedPoints.size(); i++) {
    trackingPoints[i].count++;
    trackingPoints[i].data = detectedPoints[i];
    currentFrame.matchedPoint[trackingPoints[i].id] = detectedPoints[i];
  }

  // prepare for next tracking
  for (auto pts : undetectedPoints) {
    featureId++;
    trackingPoints.emplace_back(TrackingPointsInfo{featureId, 1, pts});
    currentFrame.unmatchedPoint[featureId] = pts;
  }

  // std::cout << detectedPoints.size() << "\t" << undetectedPoints.size() <<
  // std::endl;

  hasPrediction = false;
}

double RadarTracker::calculatePDF(Frame::RadarData &previousPoint,
                                  Frame::RadarData &currentPoint) {
  double pdfR = pdf(previousPoint.range, currentPoint.range, SigmaR);
  double pdfTheta =
      pdf(previousPoint.azimuth, currentPoint.azimuth, SigmaTheta);
  double pdfPhi =
      pdf(previousPoint.elevation, currentPoint.elevation, SigmaPhi);
  return pdfR * pdfTheta * pdfPhi;
}

}  // namespace Frontend