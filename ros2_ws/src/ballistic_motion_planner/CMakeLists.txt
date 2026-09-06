cmake_minimum_required(VERSION 3.8)
project(ballistic_motion_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Pure C++ planner core — no ROS dependencies, testable standalone.
# NOTE: never add -ffast-math; speed binning relies on the default FP
# environment (round-half-to-even) to match the Python reference.
add_library(ballistic_core STATIC
  src/map2d5.cpp
  src/ballistic_physics.cpp
  src/hopping_astar.cpp)
target_compile_features(ballistic_core PUBLIC cxx_std_17)
target_include_directories(ballistic_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_executable(ballistic_planner_node src/ballistic_planner_node.cpp)
target_link_libraries(ballistic_planner_node ballistic_core)
ament_target_dependencies(ballistic_planner_node
  rclcpp grid_map_core grid_map_ros grid_map_msgs visualization_msgs geometry_msgs)

add_executable(ballistic_parity_test src/parity_main.cpp)
target_link_libraries(ballistic_parity_test ballistic_core)

install(TARGETS ballistic_planner_node ballistic_parity_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()
