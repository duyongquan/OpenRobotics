cmake_minimum_required(VERSION 3.8)
project(timer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED) 
# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  nav2_util)

# sources
set(SRCS 
  src/tutorials_timer_demo1_initial_env.cpp)

# demo1
add_executable(tutorial.timer.initial_env_test ${SRCS} src/tutorials_timer_demo1_initial_env_test.cpp)
ament_target_dependencies(tutorial.timer.initial_env_test ${DEPS})

#demo2 tutorials_timer_demo2_start_stop_test
add_executable(tutorial.timer.start_stop_test ${SRCS} src/tutorials_timer_demo2_start_stop_test.cpp)
ament_target_dependencies(tutorial.timer.start_stop_test ${DEPS})

install(TARGETS
  tutorial.timer.initial_env_test
  tutorial.timer.start_stop_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
