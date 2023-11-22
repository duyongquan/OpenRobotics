cmake_minimum_required(VERSION 3.8)
project(common_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

add_compile_options(-g -O0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Boost REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs)

# demo0 tutorials_common_utils_demo0_backward_cpp_test.cpp
# add_executable(tutorial.common_utils.backward_cpp src/tutorials_common_utils_demo0_backward_cpp_test.cpp)
# ament_target_dependencies(tutorial.common_utils.backward_cpp ${DEPS})

# demo1 tutorials_common_utils_demo1_gdb_cpp_test
add_executable(tutorial.common_utils.gdb src/tutorials_common_utils_demo1_gdb_cpp_test.cpp)
ament_target_dependencies(tutorial.common_utils.gdb ${DEPS})

# demo02 tutorial.common_utils.boost_graph
add_executable(tutorial.common_utils.boost_graph src/tutorials_common_utils_demo2_boost_graph_test.cpp)
ament_target_dependencies(tutorial.common_utils.boost_graph ${DEPS})

install(TARGETS
  # tutorial.common_utils.backward_cpp
  tutorial.common_utils.gdb 
  tutorial.common_utils.boost_graph
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
