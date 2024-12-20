#include "RadarManager.hpp"

namespace Data {
void RadarManager::push(Frame::RadarFrame frame) { data.emplace_back(frame); }

void RadarManager::popFront() {
  if (size() != 0) data.pop_front();
}

void RadarManager::popBack() {
  if (size() != 0) data.pop_back();
}

ros::Time RadarManager::getFirstTimestamp() const {
  return (size() == 0) ? ros::Time{0, 0} : data.front().receivedTime;
}

ros::Time RadarManager::getLastTimestamp() const {
  return (size() == 0) ? ros::Time{0, 0} : data.back().receivedTime;
}

size_t RadarManager::size() const { return data.size(); }

std::vector<Frame::RadarFrame> RadarManager::getDataWithinInterval(
    ros::Time startTime, ros::Time endTime) {
  std::vector<Frame::RadarFrame> result;
  for (auto &candidate : data)
    if (startTime <= candidate.receivedTime &&
        candidate.receivedTime <= endTime)
      result.emplace_back(candidate);
  return result;
}
}  // namespace Data