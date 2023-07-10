cmake_minimum_required(VERSION 3.8)
project(rviz)

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
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  interactive_markers
  visualization_msgs)

# sources
set(SRCS 
  src/tutorials_rviz_demo1_initial_env.cpp)

# demo1
add_executable(tutorial.rviz.demo1_initial_env_test ${SRCS} src/tutorials_rviz_demo1_initial_env_test.cpp)
ament_target_dependencies(tutorial.rviz.demo1_initial_env_test ${DEPS})

# basic_shapes
add_executable(tutorial.rviz.basic_shapes ${SRCS} src/tutorials_rviz_marker_basic_shapes_test.cpp)
ament_target_dependencies(tutorial.rviz.basic_shapes ${DEPS})

# points_and_lines
add_executable(tutorial.rviz.points_and_lines ${SRCS} src/tutorials_rviz_marker_points_and_lines_test.cpp)
ament_target_dependencies(tutorial.rviz.points_and_lines ${DEPS})

# simple_marker
add_executable(tutorial.rviz.simple_marker ${SRCS} src/tutorials_rviz_marker_simple_marker_test.cpp)
ament_target_dependencies(tutorial.rviz.simple_marker ${DEPS})

# cube
add_executable(tutorial.rviz.cube ${SRCS} src/tutorials_rviz_marker_cube_test.cpp)
ament_target_dependencies(tutorial.rviz.cube ${DEPS})

# eight_shape_path
add_executable(tutorial.rviz.eight_shape_path ${SRCS} src/tutorials_rviz_demo2_eight_shape_path_test.cpp)
ament_target_dependencies(tutorial.rviz.eight_shape_path ${DEPS})

# tutorials_rviz_demo4_markers_array_lines_test.cpp
add_executable(tutorial.rviz.markers_array_lines ${SRCS} src/tutorials_rviz_demo4_markers_array_lines_test.cpp)
ament_target_dependencies(tutorial.rviz.markers_array_lines ${DEPS})


install(TARGETS
  tutorial.rviz.demo1_initial_env_test
  tutorial.rviz.basic_shapes
  tutorial.rviz.points_and_lines
  tutorial.rviz.simple_marker
  tutorial.rviz.cube
  tutorial.rviz.eight_shape_path
  tutorial.rviz.markers_array_lines
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
