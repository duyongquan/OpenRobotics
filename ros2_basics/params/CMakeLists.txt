cmake_minimum_required(VERSION 3.8)
project(params)

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
  src/tutorials_params_demo1_yaml_config.cpp)

# demo1 tutorials_params_demo1_yaml_config_test.cpp
add_executable(tutorial.params.yaml_config_test ${SRCS} src/tutorials_params_demo1_yaml_config_test.cpp)
ament_target_dependencies(tutorial.params.yaml_config_test ${DEPS})

# demo2 tutorials_params_demo2_nav_bringup_yaml_test.cpp
add_executable(tutorial.params.nav_bringup_yaml_test ${SRCS} src/tutorials_params_demo2_nav_bringup_yaml_test.cpp)
ament_target_dependencies(tutorial.params.nav_bringup_yaml_test ${DEPS})

install(TARGETS
  tutorial.params.yaml_config_test
  tutorial.params.nav_bringup_yaml_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
