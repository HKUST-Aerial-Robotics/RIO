#include "rosWarper.hpp"

#ifdef DEBUG
#define BACKWARD_HAS_BFD 1
#define BACKWARD_HAS_DW 1
#include <backward.hpp>
backward::SignalHandling sh;
#endif

int main(int argc, char **argv) {
  ros::init(argc, argv, "rio");
  ros::NodeHandle nodeHandle("~");

  std::shared_ptr<RIO> rio = std::make_shared<RIO>(nodeHandle);
  ros::spin();
  return 0;
}