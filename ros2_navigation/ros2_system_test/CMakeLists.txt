cmake_minimum_required(VERSION 3.8)
project(ros2_system_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline REQUIRED) 
find_package(trajectory REQUIRED) 
find_package(visualization_tools REQUIRED) 
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
  bspline
  trajectory
  visualization_tools
)

# test bspline
add_executable(test_non_uniform_bspline src/test_non_uniform_bspline.cpp)
ament_target_dependencies(test_non_uniform_bspline ${dependencies})


install(TARGETS test_non_uniform_bspline
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
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
