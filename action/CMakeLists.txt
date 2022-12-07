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
find_package(tutorials_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

# include
include_directories(include)

# dependency
set(DEPS 
  rclcpp
  rclcpp_action
  std_msgs
  tutorials_msgs)

# sources
set(SRCS 
  src/tutorials_action_demo1_member_functions.cpp)

# demo1 tutorials_action_demo1_member_functions_client_test
add_executable(tutorial.action.member_functions_client_test ${SRCS} src/tutorials_action_demo1_member_functions_client_test.cpp ${SRCS})
ament_target_dependencies(tutorial.action.member_functions_client_test ${DEPS})

# demo1 tutorials_action_demo1_member_functions_server_test
add_executable(tutorial.action.member_functions_server_test ${SRCS} src/tutorials_action_demo1_member_functions_server_test.cpp ${SRCS})
ament_target_dependencies(tutorial.action.member_functions_server_test ${DEPS})

install(TARGETS
  tutorial.action.member_functions_client_test
  tutorial.action.member_functions_server_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
