cmake_minimum_required(VERSION 3.8)
project(service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tutorials_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  tutorials_msgs)

# sources
set(SRCS 
  src/tutorials_service_demo1_initial_env.cpp)

# demo1
add_executable(tutorial.service.initial_env_test ${SRCS} src/tutorials_service_demo1_initial_env_test.cpp)
ament_target_dependencies(tutorial.service.initial_env_test ${DEPS})

install(TARGETS
  tutorial.service.initial_env_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
