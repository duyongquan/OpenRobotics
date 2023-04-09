cmake_minimum_required(VERSION 3.8)
project(pedsim_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(pedsim_common REQUIRED)
# find_package(pedsim_utils REQUIRED)
find_package(pedsim_msgs REQUIRED)


set(dependencies
  rclcpp
  tf2
  tf2_eigen
  tf2_msgs
  nav_msgs
  tf2_ros
  sensor_msgs
  # pedsim_utils
  pedsim_msgs
)

pedsim_package()
include_directories(include)

# People point cloud sensor.
set(PEOPLE_PCD_EXEC_NAME pedsim_people_sensor)
add_executable(${PEOPLE_PCD_EXEC_NAME} src/people_point_cloud.cpp)
ament_target_dependencies(${PEOPLE_PCD_EXEC_NAME} ${dependencies})

# # Obstacle point cloud sensor.
# set(OBSTACLE_PCD_EXEC_NAME pedsim_obstacle_sensor)
# add_executable(${OBSTACLE_PCD_EXEC_NAME} src/pedsim_sensors/obstacle_point_cloud.cpp)
# add_dependencies(${OBSTACLE_PCD_EXEC_NAME} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${OBSTACLE_PCD_EXEC_NAME} ${catkin_LIBRARIES})

# # Point cloud sensor.
# set(OCCLUSION_PCD_EXEC_NAME pedsim_occlusion_sensor)
# add_executable(${OCCLUSION_PCD_EXEC_NAME} src/pedsim_sensors/occlusion_point_cloud.cpp)
# add_dependencies(${OCCLUSION_PCD_EXEC_NAME} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${OCCLUSION_PCD_EXEC_NAME} ${catkin_LIBRARIES})

install(TARGETS
  ${PEOPLE_PCD_EXEC_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
