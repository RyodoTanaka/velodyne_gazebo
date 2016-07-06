cmake_minimum_required(VERSION 2.8.3)
project(velodyne_gazebo_plugin)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  sensor_msgs
  gazebo_ros
  )

find_package(gazebo REQUIRED)
include_directories(include)

# Find Gazebo
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11")

# Build our plugin
add_library(velodyne_plugin SHARED src/velodyne_plugin.cpp)
target_link_libraries(velodyne_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} pthread)
