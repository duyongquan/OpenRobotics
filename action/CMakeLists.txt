cmake_minimum_required(VERSION 3.8)
project(action)

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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorials_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  rclcpp_action
  std_msgs
  tutorials_msgs)

# sources
set(SRCS 
  src/tutorials_action_demo1_member_functions.cpp)

add_library(tutorial_action SHARED ${SRCS})
target_link_libraries(tutorial_action ${DEPS})

# demo1
# add_executable(tutorial.action.initial_env_test ${SRCS} src/tutorials_action_demo1_member_functions.cpp)
# ament_target_dependencies(tutorial.topic.initial_env_test ${DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
