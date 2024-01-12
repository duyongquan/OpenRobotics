cmake_minimum_required(VERSION 3.8)
project(sdf_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
  image_transport
  cv_bridge
)

add_library(${library_name} SHARED
  include/${PROJECT_NAME}/collision_map.hpp
  include/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.hpp
  include/${PROJECT_NAME}/sdf.hpp
  src/${PROJECT_NAME}/collision_map.cpp
  src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
  src/${PROJECT_NAME}/sdf.cpp
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
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()