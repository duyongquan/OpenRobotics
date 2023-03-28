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
find_package(rcutils)

# include
include_directories(
  include
  tools
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  rcutils)

# sources
set(SRCS 
  src/tutorials_topic_demo1_initial_env.cpp
  src/tutorials_topic_demo2_member_function.cpp
  src/tutorials_topic_demo3_lambda.cpp
  src/tutorials_topic_demo5_member_function_with_topic_statistics.cpp
  src/tutorials_topic_demo6_member_function_with_unique_network_flow_endpoints.cpp)

# demo1
add_executable(tutorial.topic.initial_env_test ${SRCS} src/tutorials_topic_demo1_initial_env_test.cpp)
ament_target_dependencies(tutorial.topic.initial_env_test ${DEPS})

# demo2 tutorials_topic_demo2_member_function_test
add_executable(tutorial.topic.member_function_test ${SRCS} src/tutorials_topic_demo2_member_function_test.cpp)
ament_target_dependencies(tutorial.topic.member_function_test ${DEPS})

# demo3 tutorials_topic_demo3_lambda_test
add_executable(tutorial.topic.lambda_test ${SRCS} src/tutorials_topic_demo3_lambda_test.cpp)
ament_target_dependencies(tutorial.topic.lambda_test ${DEPS})

# demo4 tutorials_topic_demo4_not_composable_test
add_executable(tutorial.topic.not_composable_test src/tutorials_topic_demo4_not_composable_test.cpp)
ament_target_dependencies(tutorial.topic.not_composable_test ${DEPS})

# demo5 tutorials_topic_demo5_member_function_with_topic_statistics_test
add_executable(tutorials.topic.member_function_with_topic_statistics_test ${SRCS}
  src/tutorials_topic_demo5_member_function_with_topic_statistics_test.cpp)
ament_target_dependencies(tutorials.topic.member_function_with_topic_statistics_test ${DEPS})

# demo6 tutorials_topic_demo6_member_function_with_unique_network_flow_endpoints_test
add_executable(tutorials.topic.member_function_with_unique_network_flow_endpoints_test ${SRCS}
  src/tutorials_topic_demo6_member_function_with_unique_network_flow_endpoints_test.cpp)
ament_target_dependencies(tutorials.topic.member_function_with_unique_network_flow_endpoints_test ${DEPS})

# demo6 grpc_tool
add_executable(tutorials.topic.grpc_tool tools/grpc_tool.cpp)
ament_target_dependencies(tutorials.topic.grpc_tool ${DEPS})

install(TARGETS
  tutorial.topic.initial_env_test
  tutorial.topic.member_function_test
  tutorial.topic.lambda_test
  tutorial.topic.not_composable_test
  tutorials.topic.member_function_with_topic_statistics_test
  tutorials.topic.member_function_with_unique_network_flow_endpoints_test
  tutorials.topic.grpc_tool
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()