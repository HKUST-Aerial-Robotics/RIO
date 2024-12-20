#include "ImuPreintegrationManager.hpp"

namespace Factor {
void ImuPreintegrationManger::pushBack(Residuals::Preintegration data,
                                       int startIndex, int endIndex) {
  imuPreintegration.emplace_back(data);
  relations.emplace_back(Relation{startIndex, endIndex});
}

void ImuPreintegrationManger::popBack() {
  imuPreintegration.pop_back();
  relations.pop_back();
}

void ImuPreintegrationManger::popFront() {
  imuPreintegration.pop_front();
  relations.pop_front();
}

void ImuPreintegrationManger::shiftStateIndex(int number) {
  for (auto &data : relations) {
    data.startStateIndex -= number;
    data.endStateIndex -= number;
  }
}

}  // namespace Factor