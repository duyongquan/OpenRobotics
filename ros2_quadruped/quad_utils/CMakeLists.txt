cmake_minimum_required(VERSION 3.8)
project(quad_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# RBDL
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

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
find_package(PCL 1.12 REQUIRED) ## See ROS REP2000 for expected PCL version
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# include
include_directories(include)
include_directories(${RBDL_INCLUDE_DIR})

# Add links to PCL definitons and directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add links to PCL definitons and directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# dependency
set(dependencies 
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
  src/rviz_interface.cpp
  src/rviz_interface_node.cpp
  src/terrain_map_publisher.cpp
  src/trajectory_publisher.cpp)

# Declare a cpp library
set(library_name quad_utils)
add_library(${library_name} SHARED ${SRCS})
ament_target_dependencies(${library_name} ${dependencies})
target_link_libraries(${library_name}
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
)

# quad_utils_node
add_executable(quad_utils_node src/main.cpp ${SRCS})
ament_target_dependencies(quad_utils_node ${dependencies})
target_link_libraries(quad_utils_node
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
)

install(TARGETS
  quad_utils_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()