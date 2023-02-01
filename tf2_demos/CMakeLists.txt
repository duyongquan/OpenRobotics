cmake_minimum_required(VERSION 3.8)
project(tf2_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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
  tf2
  tf2_ros)

# sources
set(SRCS 
  src/tutorials_tf2_demo1_can_transform_test.cpp)

# demo1
add_executable(tutorial.tf2.can_transform_test ${SRCS} src/tutorials_tf2_demo1_can_transform_test.cpp)
ament_target_dependencies(tutorial.tf2.can_transform_test ${DEPS})

install(TARGETS
  tutorial.tf2.can_transform_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()