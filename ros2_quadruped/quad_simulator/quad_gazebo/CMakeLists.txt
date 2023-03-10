cmake_minimum_required(VERSION 3.8)
project(quad_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quad_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(xacro REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  quad_msgs
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
  xacro)

# srcs
set(SRCS 
  src/contact_state_publisher.cpp)

# gazebo_ros_contact_sensor
add_library(gazebo_ros_contact_sensor SHARED src/contact_plugin.cpp)
ament_target_dependencies(gazebo_ros_contact_sensor ${DEPS})

# contact_state_publisher
add_executable(contact_state_publisher src/main.cpp ${SRCS})
ament_target_dependencies(contact_state_publisher ${DEPS})

install(TARGETS
  contact_state_publisher
  gazebo_ros_contact_sensor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(gazebo_ros_contact_sensor)
ament_package()