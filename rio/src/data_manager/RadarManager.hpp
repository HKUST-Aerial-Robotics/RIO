#pragma once

#include <deque>
#include <memory>

#include "../frame_type/RadarFrame.hpp"
#include "ros/time.h"

namespace Data {

class RadarManager {
 public:
  RadarManager() = default;
  ~RadarManager() = default;

  RadarManager(RadarManager &another) = delete;
  RadarManager(RadarManager &&another) = delete;
  RadarManager &operator=(const RadarManager &another) = delete;
  RadarManager &operator=(RadarManager &&another) = delete;

  void push(Frame::RadarFrame frame);
  void popFront();
  void popBack();

  ros::Time getFirstTimestamp() const;
  ros::Time getLastTimestamp() const;
  size_t size() const;

  std::vector<Frame::RadarFrame> getDataWithinInterval(ros::Time startTime,
                                                       ros::Time endTime);

  std::deque<Frame::RadarFrame> data;
};
}  // namespace Data