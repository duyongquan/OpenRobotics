cmake_minimum_required(VERSION 3.8)
project(topic)

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

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs)

# sources
set(SRCS 
  src/tutorials_topic_demo1_initial_env.cpp)

add_executable(tutorial.topic.initial_env_test ${SRCS} src/tutorials_topic_demo1_initial_env_test.cpp)
ament_target_dependencies(tutorial.topic.initial_env_test ${DEPS})

# add_executable(publisher_member_function member_function.cpp)
# ament_target_dependencies(publisher_member_function rclcpp std_msgs)

# add_executable(publisher_member_function_with_unique_network_flow_endpoints member_function_with_unique_network_flow_endpoints.cpp)
# ament_target_dependencies(publisher_member_function_with_unique_network_flow_endpoints rclcpp std_msgs)

# add_executable(publisher_not_composable not_composable.cpp)
# ament_target_dependencies(publisher_not_composable rclcpp std_msgs)

install(TARGETS
  tutorial.topic.initial_env_test
  # publisher_member_function
  # publisher_member_function_with_unique_network_flow_endpoints
  # publisher_not_composable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()