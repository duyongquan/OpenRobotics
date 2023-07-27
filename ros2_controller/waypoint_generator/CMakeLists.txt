cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros2_controller_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  std_msgs
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  ros2_controller_msgs
  interactive_markers
  visualization_msgs)

# sources
set(SRCS 
  src/waypoint_generator.cpp)

# ros2_controller.motion_model.vehicle_position_publisher
add_executable(ros2_controller.waypoint_generator src/waypoint_generator_node.cpp ${SRCS})
ament_target_dependencies(ros2_controller.waypoint_generator ${dependencies})

install(TARGETS
  ros2_controller.waypoint_generator
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
