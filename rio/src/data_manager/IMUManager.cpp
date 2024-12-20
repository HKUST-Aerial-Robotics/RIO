#include "IMUManager.hpp"

#include <tuple>
#include <vector>

namespace Data {
void IMUManager::push(Frame::IMUFrame frame) { data.emplace_back(frame); }

void IMUManager::popFront() {
  if (size() != 0) data.pop_front();
}

void IMUManager::popBack() {
  if (size() != 0) data.pop_back();
}

ros::Time IMUManager::getFirstTimestamp() const {
  return (size() == 0) ? ros::Time{0, 0} : data.front().receivedTime;
}

ros::Time IMUManager::getLastTimestamp() const {
  return (size() == 0) ? ros::Time{0, 0} : data.back().receivedTime;
}

size_t IMUManager::size() const { return data.size(); }

std::vector<Frame::IMUFrame> IMUManager::getDataWithinInterval(
    ros::Time startTime, ros::Time endTime) {
  std::vector<Frame::IMUFrame> result;
  for (auto &candidate : data)
    if (startTime <= candidate.receivedTime &&
        candidate.receivedTime <= endTime)
      result.emplace_back(candidate);
  return result;
}

};  // namespace Data