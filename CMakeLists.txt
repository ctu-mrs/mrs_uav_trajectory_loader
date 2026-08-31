cmake_minimum_required(VERSION 3.28)
project(mrs_uav_trajectory_loader)

# C++20 (matching path_loader)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# ---------------- Dependencies ----------------
set(DEPENDENCIES rclcpp rclcpp_components mrs_msgs std_srvs mrs_lib)

foreach(dep IN LISTS DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

# ---------------- Build ----------------
add_library(trajectory_loader_component SHARED src/trajectory_loader.cpp)

target_link_libraries(trajectory_loader_component
  rclcpp::rclcpp
  rclcpp_components::component
  ${mrs_msgs_TARGETS}
  ${std_srvs_TARGETS}
  mrs_lib::mrs_lib
)

# register as component and add standalone executable
rclcpp_components_register_node(
  trajectory_loader_component PLUGIN
  "mrs_uav_trajectory_loader::TrajectoryLoader" EXECUTABLE
  trajectory_loader_node_component)

# ---------------- Install ---------------------
install(
  TARGETS trajectory_loader_component trajectory_loader_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()
