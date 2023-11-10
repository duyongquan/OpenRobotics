cmake_minimum_required(VERSION 3.8)
project(robotic_assets)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY resources 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
