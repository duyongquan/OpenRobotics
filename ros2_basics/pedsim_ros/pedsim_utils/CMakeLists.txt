cmake_minimum_required(VERSION 3.5)
project(pedsim_utils)

find_package(ament_cmake REQUIRED)
find_package(pedsim_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pedsim_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies
  nav2_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  pedsim_msgs
)

pedsim_package()

include_directories(include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/geometry.cpp
  src/pedsim_utils.cpp
)
ament_target_dependencies(${library_name} ${dependencies})

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
