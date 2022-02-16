cmake_minimum_required(VERSION 3.1.2)
project(trajectory_loader)

find_package(catkin REQUIRED COMPONENTS
  mrs_msgs
  roscpp
  rospy
  std_msgs
  mrs_lib
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECUTABLES
  trajectory_loader
  )

catkin_package()

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

install(DIRECTORY ./
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.xml"
  )
