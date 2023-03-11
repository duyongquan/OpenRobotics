cmake_minimum_required(VERSION 3.8)
project(quad_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  quad_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs)

# srcs
set(SRCS 
  src/fast_terrain_map.cpp
  src/math_utils.cpp
  src/mesh_to_grid_map_converter.cpp
  src/quad_kd.cpp
  src/ros_utils.cpp
  src/terrain_map_publisher.cpp
  src/trajectory_publisher.cpp)

# quad_utils_node
add_executable(quad_utils_node src/main.cpp ${SRCS})
ament_target_dependencies(quad_utils_node ${DEPS})

install(TARGETS
  quad_utils_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
