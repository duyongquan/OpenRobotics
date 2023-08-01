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
find_package(quad_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  nav_msgs
  quad_msgs
  sensor_msgs
  visualization_msgs
  grid_map_core
  grid_map_ros
  ament_index_cpp
  tf2
  tf2_ros)

# sources
set(SRCS
  src/fast_terrain_map.cpp
  src/global_body_planner.cpp
  src/graph_class.cpp
  src/planner_class.cpp
  src/planning_utils.cpp
  src/rrt_connect.cpp
  src/rrt_star_connect.cpp
  src/rrt.cpp
  src/rviz_interface.cpp
  src/terrain_map_publisher.cpp)

# terrain_map_publisher_node
add_executable(terrain_map_publisher_node ${SRCS} src/terrain_map_publisher_node.cpp ${SRCS})
ament_target_dependencies(terrain_map_publisher_node ${DEPS})

# rviz_interface_node
add_executable(rviz_interface_node ${SRCS} src/rviz_interface_node.cpp ${SRCS})
ament_target_dependencies(rviz_interface_node ${DEPS})

# global_body_planner_node
add_executable(global_body_planner_node ${SRCS} src/global_body_planner_node.cpp ${SRCS})
ament_target_dependencies(global_body_planner_node ${DEPS})

install(TARGETS
  terrain_map_publisher_node
  rviz_interface_node
  global_body_planner_node
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
