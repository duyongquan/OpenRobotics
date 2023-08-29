cmake_minimum_required(VERSION 3.8)
project(map_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  std_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav2_msgs
  nav_msgs)

# sources
set(SRCS 
  src/maps.cpp
  src/perlinnoise.cpp
  src/random_forest_sensing.cpp)

# map_generator
add_executable(ros2_navigation.map_generator src/map_generator_node.cpp ${SRCS})
ament_target_dependencies(ros2_navigation.map_generator ${dependencies})

# random_forest_sensing_node
add_executable(ros2_navigation.random_forest_sensing src/random_forest_sensing_node.cpp ${SRCS})
ament_target_dependencies(ros2_navigation.random_forest_sensing ${dependencies})

install(TARGETS
  ros2_navigation.map_generator
  ros2_navigation.random_forest_sensing
  DESTINATION lib/${PROJECT_NAME}
)

# install
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
