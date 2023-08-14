cmake_minimum_required(VERSION 3.5)
project(mrs_uav_trajectory_loader)

set(CATKIN_DEPENDENCIES
  cmake_modules
  mrs_msgs
  roscpp
  rospy
  std_msgs
  mrs_lib
  )

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_DEPENDENCIES}
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLES
  trajectory_loader
  )

catkin_package(
  DEPENDS ${CATKIN_DEPENDENCIES}
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

## Declare a C++ executable
add_executable(trajectory_loader
  src/trajectory_loader.cpp
  )

add_dependencies(trajectory_loader
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(trajectory_loader
  ${catkin_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${EXECUTABLES}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch config sample_trajectories
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
