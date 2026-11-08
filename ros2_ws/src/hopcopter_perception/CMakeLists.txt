cmake_minimum_required(VERSION 3.8)
project(hopcopter_perception)

# The mapper processes 20 k points per scan: build optimized unless told otherwise.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(elevation_map_publisher REQUIRED)
# grid_map_core's find_package adds its Eigen plugins (EIGEN_*_PLUGIN) to every
# target in this directory, so every target that includes Eigen needs its headers.
get_target_property(_grid_map_core_include grid_map_core::grid_map_core INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${_grid_map_core_include})

# The mapper's core: Eigen only, no ROS (mapper_check drives it offline).
add_library(elevation_mapper_core STATIC src/elevation_mapper.cpp)
target_compile_features(elevation_mapper_core PUBLIC cxx_std_17)
target_include_directories(elevation_mapper_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(elevation_mapper_core Eigen3::Eigen)

add_executable(lidar_elevation_mapper src/lidar_elevation_mapper_node.cpp)
target_link_libraries(lidar_elevation_mapper elevation_mapper_core)
ament_target_dependencies(lidar_elevation_mapper
  rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_eigen grid_map_core grid_map_ros grid_map_msgs)

add_executable(lidar_sim_adapter src/lidar_sim_adapter.cpp)
ament_target_dependencies(lidar_sim_adapter rclcpp sensor_msgs)

add_executable(gz_truth_tf src/gz_truth_tf.cpp)
ament_target_dependencies(gz_truth_tf rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(lio_anchor src/lio_anchor.cpp)
ament_target_dependencies(lio_anchor
  rclcpp nav_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_eigen)

# Offline check of the mapper on synthetic scans of every scenario (no ROS graph).
add_executable(mapper_check src/mapper_check_main.cpp)
target_link_libraries(mapper_check elevation_mapper_core elevation_map_publisher::scenarios)

install(TARGETS lidar_elevation_mapper lidar_sim_adapter gz_truth_tf lio_anchor mapper_check
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/plot_elevation_map.py scripts/lio_report.py scripts/map_report.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
