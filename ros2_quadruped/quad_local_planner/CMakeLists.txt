cmake_minimum_required(VERSION 3.8)
project(quad_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(quad_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quad_nmpc_controller REQUIRED)


# include
include_directories(
  include
)

# Add links to PCL definitons and directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# dependency
set(DEPS 
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  quad_msgs
  grid_map_core
  grid_map_ros
  quad_utils
  tf2
  tf2_ros
  quad_nmpc_controller)

# sources
set(SRCS
  src/local_planner.cpp
  src/local_footstep_planner.cpp)


# local_planner_node
add_executable(local_planner_node ${SRCS} src/local_planner_node.cpp ${SRCS})
ament_target_dependencies(local_planner_node ${DEPS})

# install
install(
  TARGETS 
    local_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
