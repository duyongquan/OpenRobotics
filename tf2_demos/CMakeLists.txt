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

# demo1
add_executable(tutorial.tf2.can_transform_test src/tutorials_tf2_demo1_can_transform_test.cpp)
ament_target_dependencies(tutorial.tf2.can_transform_test ${DEPS})

#demo2 tutorials_tf2_demo2_dynamic_tf_transform_test.cpp
add_executable(tutorial.tf2.dynamic_tf_transform_test src/tutorials_tf2_demo2_dynamic_tf_transform_test.cpp)
ament_target_dependencies(tutorial.tf2.dynamic_tf_transform_test ${DEPS})

install(TARGETS
  tutorial.tf2.can_transform_test
  tutorial.tf2.dynamic_tf_transform_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()