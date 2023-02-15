cmake_minimum_required(VERSION 3.8)
project(nav2_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${EIGEN3_INCLUDE_DIR})

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav2_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
  ${Boost_LIBRARIES})

# SRCS
set(SRCS 
  src/tutorials_nav2_utils_poses_publisher.cpp)

# demo1
add_executable(tutorial.nav2.b_spline_path_test src/tutorials_nav2_demo1_b_spline_path_test.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2.b_spline_path_test ${DEPS})

# demo2

# demo3
add_executable(tutorial.nav2.cubic_spline_path_test src/tutorials_nav2_demo3_cubic_spline_planner_test.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2.cubic_spline_path_test ${DEPS})

# demo4 tutorials_nav2_demo4_bezier_path_test.cpp
add_executable(tutorial.nav2.bezier_path src/tutorials_nav2_demo4_bezier_path_test.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2.bezier_path ${DEPS})

install(TARGETS
  tutorial.nav2.b_spline_path_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tutorial.nav2.cubic_spline_path_test
  tutorial.nav2.bezier_path
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
