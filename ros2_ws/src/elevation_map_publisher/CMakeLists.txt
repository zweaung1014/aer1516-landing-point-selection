cmake_minimum_required(VERSION 3.8)
project(elevation_map_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

# The scenarios' known maps, header-only and ROS-free: also used by the planner's
# ballistic_plan_check and hopcopter_perception's mapper_check.
add_library(scenarios INTERFACE)
target_include_directories(scenarios INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS scenarios EXPORT export_${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)

add_executable(elevation_map_publisher_node src/elevation_map_publisher_node.cpp)
target_compile_features(elevation_map_publisher_node PUBLIC cxx_std_17)
target_link_libraries(elevation_map_publisher_node scenarios)
ament_target_dependencies(elevation_map_publisher_node
  rclcpp grid_map_core grid_map_ros grid_map_msgs)

install(TARGETS elevation_map_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
