cmake_minimum_required(VERSION 3.8)
project(common_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(backward_ros REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs)

# demo0 tutorials_common_utils_demo0_backward_cpp_test.cpp
add_executable(tutorial.common_utils.backward_cpp src/tutorials_common_utils_demo0_backward_cpp_test.cpp)
ament_target_dependencies(tutorial.common_utils.backward_cpp ${DEPS})

install(TARGETS
  tutorial.common_utils.backward_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# install launch directory
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
