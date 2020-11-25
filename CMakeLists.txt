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

add_compile_options("-std=c++17")
add_compile_options("-Wall")

catkin_package()

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(trajectory_loader src/trajectory_loader.cpp)
add_dependencies(trajectory_loader ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_loader
  ${catkin_LIBRARIES}
  )

