cmake_minimum_required(VERSION 3.8)
project(quad_global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  grid_map_core
  grid_map_ros
  ament_index_cpp)

# sources
set(SRCS 
  src/terrain_map_publisher.cpp)

# terrain_map_publisher_node
add_executable(terrain_map_publisher_node ${SRCS} src/terrain_map_publisher_node.cpp ${SRCS})
ament_target_dependencies(terrain_map_publisher_node ${DEPS})

install(TARGETS
  terrain_map_publisher_node
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
