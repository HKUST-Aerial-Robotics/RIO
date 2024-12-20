#pragma once

#include <deque>
#include <memory>

#include "../frame_type/IMUFrame.hpp"
#include "ros/time.h"

namespace Data {

class IMUManager {
 public:
  IMUManager() = default;
  ~IMUManager() = default;

  IMUManager(IMUManager &another) = delete;
  IMUManager(IMUManager &&another) = delete;
  IMUManager &operator=(const IMUManager &another) = delete;
  IMUManager &operator=(IMUManager &&another) = delete;

  void push(Frame::IMUFrame frame);
  void popFront();
  void popBack();

  ros::Time getFirstTimestamp() const;
  ros::Time getLastTimestamp() const;
  size_t size() const;

  std::vector<Frame::IMUFrame> getDataWithinInterval(ros::Time startTime,
                                                     ros::Time endTime);

  std::deque<Frame::IMUFrame> data;
};
}  // namespace Data