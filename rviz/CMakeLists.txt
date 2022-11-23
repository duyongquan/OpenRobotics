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
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  nav2_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs)

# sources
set(SRCS 
  src/tutorials_rviz_demo1_initial_env.cpp)

# demo1
add_executable(tutorial.topic.demo1_initial_env_test ${SRCS} src/tutorials_rviz_demo1_initial_env_test.cpp)
ament_target_dependencies(tutorial.topic.demo1_initial_env_test ${DEPS})

install(TARGETS
  tutorial.topic.demo1_initial_env_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
