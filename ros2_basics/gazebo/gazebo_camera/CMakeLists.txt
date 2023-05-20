cmake_minimum_required(VERSION 3.8)
project(gazebo_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gazebo_camera src/gazebo_camera.cpp)
target_include_directories(gazebo_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gazebo_camera PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS gazebo_camera
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY models DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
