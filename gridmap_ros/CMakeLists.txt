cmake_minimum_required(VERSION 3.8)
project(gridmap_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(filters REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  nav2_msgs
  cv_bridge
  filters
  grid_map_core
  grid_map_cv
  grid_map_msgs
  grid_map_ros)

# sources
set(SRCS 
  src/tutorials_gridmap_demo1_opencv.cpp)

# demo1
add_executable(tutorial.gridmap.opencv_test ${SRCS} src/tutorials_gridmap_demo1_opencv_test.cpp)
ament_target_dependencies(tutorial.gridmap.opencv_test ${dependencies})

install(TARGETS
  tutorial.gridmap.opencv_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
