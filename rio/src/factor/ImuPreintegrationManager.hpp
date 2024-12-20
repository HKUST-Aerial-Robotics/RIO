#pragma once
#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "Preintegration.hpp"

namespace Factor {

class ImuPreintegrationManger {
 public:
  struct Relation {
    int startStateIndex;
    int endStateIndex;
  };

  std::deque<Residuals::Preintegration> imuPreintegration;
  std::deque<Relation> relations;
  void pushBack(Residuals::Preintegration data, int startIndex, int endIndex);
  void popBack();
  void popFront();
  void shiftStateIndex(int number);
};

}  // namespace Factor