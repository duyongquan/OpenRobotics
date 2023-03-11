cmake_minimum_required(VERSION 3.8)
project(quad_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(quad_gazebo REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED) ## See ROS REP2000 for expected PCL version
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)


# Add links to PCL definitons and directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# dependency
set(DEPS 
  rclcpp
  quad_msgs
  quad_gazebo
  geometry_msgs
  grid_map_msgs
  grid_map_core
  grid_map_ros
  grid_map_pcl
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  pcl_ros
  pcl_conversions)

# srcs
set(SRCS 
  src/fast_terrain_map.cpp
  src/math_utils.cpp
  src/mesh_to_grid_map_converter.cpp
  src/quad_kd.cpp
  src/ros_utils.cpp
  src/terrain_map_publisher.cpp
  src/trajectory_publisher.cpp)

## Declare a cpp library
add_library(quad_utils_core SHARED ${SRCS})
ament_target_dependencies(quad_utils_core ${DEPS})

# quad_utils_node
add_executable(quad_utils_node src/main.cpp ${SRCS})
ament_target_dependencies(quad_utils_node ${DEPS})

install(TARGETS
  quad_utils_core
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
