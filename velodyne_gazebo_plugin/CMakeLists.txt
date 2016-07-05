cmake_minimum_required(VERSION 2.8.3)
project(velodyne_gazebo_plugin)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  gazebo_ros
  )

find_package(gazebo REQUIRED)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -std=c++11")

# Build our plugin
add_library(velodyne_plugin SHARED src/velodyne_plugin.cpp)
target_link_libraries(velodyne_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES} pthread)

# Build the stand-alone test program
add_executable(vel src/velodyne.cpp)

if (${gazebo_VERSION_MAJOR} LESS 6)
  include(FindBoost)
  find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  target_link_libraries(vel ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
else()
  target_link_libraries(vel ${GAZEBO_LIBRARIES})
endif()
