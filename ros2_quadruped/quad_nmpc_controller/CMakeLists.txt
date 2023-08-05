cmake_minimum_required(VERSION 3.8)
project(quad_nmpc_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(quad_utils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  quad_msgs
  grid_map_core
  grid_map_ros
  quad_utils
  tf2
  tf2_ros)

# sources
set(SRCS
  src/quad_nlp.cpp
  src/nmpc_controller.cpp)

# Declare a cpp library
set(library_name quad_nmpc_controller)
add_library(${library_name} SHARED ${SRCS})
ament_target_dependencies(${library_name} ${dependencies})

# install
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()