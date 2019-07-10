cmake_minimum_required(VERSION 2.8.3)
project(trajectory_loader)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mrs_msgs
  roscpp
  rospy
  std_msgs
  mrs_lib
)

find_package(Boost COMPONENTS program_options thread REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fno-diagnostics-color -g -O0")

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(load_trajectory src/trajectory_loader.cpp)
add_dependencies(load_trajectory ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_trajectory
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

