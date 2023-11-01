cmake_minimum_required(VERSION 3.8)
project(visualization_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2 REQUIRED) 
find_package(tf2_ros REQUIRED) 
find_package(tf2_geometry_msgs REQUIRED) 
find_package(bspline REQUIRED) 
find_package(trajectory REQUIRED) 
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_definitions(${EIGEN3_DEFINITIONS})

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
  tf2
  bspline
  trajectory
)

set(library_name ${PROJECT_NAME})
add_library(${library_name} SHARED
  src/ros_visualization_tools.cpp
  src/planning_visualization.cpp
  src/color.cpp
)

ament_target_dependencies(${library_name} ${dependencies})

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
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
