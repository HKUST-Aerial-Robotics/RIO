cmake_minimum_required(VERSION 3.0.2)
project(rio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-std=c++17 -O3)

find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    sensor_msgs
    nav_msgs)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Backward)

catkin_package()

###########
## Build ##
###########

FILE(GLOB_RECURSE RIO_SOURCE_FILES "src/*.cpp")
add_executable(rio
    node/rio.cpp
    node/rosWarper.cpp
    ${RIO_SOURCE_FILES}
)

target_include_directories(rio
PRIVATE
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    node
    src/data_manager
    src/factor
    src/frame_type
    src/frontend
    src/residual
    src/state
    src/utility
    src/manifold
)

target_link_libraries(rio
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
${CERES_LIBRARIES}
${PCL_LIBRARIES}
-lbfd -ldw
)

