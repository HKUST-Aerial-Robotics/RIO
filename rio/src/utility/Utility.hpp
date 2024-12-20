#pragma once
#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"

template <typename T>
inline void reduceVector(std::vector<T> &vector, std::vector<uchar> status) {
  int size = 0;
  for (int i = 0; i < int(vector.size()); i++)
    if (status[i]) vector[size++] = vector[i];
  vector.resize(size);
}

inline double calDistance(cv::Point2f &point1, cv::Point2f &point2) {
  double deltaX = point1.x - point2.x;
  double deltaY = point1.y - point2.y;
  return sqrt(deltaX * deltaX + deltaY * deltaY);
}

inline double calDistance(cv::Point3f &point1, cv::Point3f &point2) {
  return std::sqrt((point1.x - point2.x) * (point1.x - point2.x) +
                   (point1.y - point2.y) * (point1.y - point2.y) +
                   (point1.z - point2.z) * (point1.z - point2.z));
}

inline double calDistance(Eigen::Vector3d &point1, Eigen::Vector3d &point2) {
  return std::sqrt((point1.x() - point2.x()) * (point1.x() - point2.x()) +
                   (point1.y() - point2.y()) * (point1.y() - point2.y()) +
                   (point1.z() - point2.z()) * (point1.z() - point2.z()));
}