cmake_minimum_required(VERSION 3.0.2)
project(dock_scan_matching)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  laser_geometry
  tf
  pcl_ros
)

find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp sensor_msgs laser_geometry tf
  DEPENDS system_lib
  DEPENDS EIGEN3
)

SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(laser_scan_processor src/main.cpp src/LaserScanProcessor.cpp)
target_link_libraries(laser_scan_processor ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
