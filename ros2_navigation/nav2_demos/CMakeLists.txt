cmake_minimum_required(VERSION 3.8)
project(nav2_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}) 
include_directories(${EIGEN3_INCLUDE_DIR})

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav2_msgs
  nav_msgs
  nav2_costmap_2d
  nav2_map_server
  nav2_planner
  nav2_navfn_planner
  nav2_controller
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  visualization_msgs
  grid_map_ros
  grid_map_core
  grid_map_cv
  grid_map_msgs
  ${Boost_LIBRARIES})

configure_file(
  ${PROJECT_SOURCE_DIR}/include/nav2_demos/config.h.cmake
  ${PROJECT_SOURCE_DIR}/include/nav2_demos/config.h)

# SRCS
set(SRCS 
  src/tutorials_nav2_utils_poses_publisher.cpp
  src/tutorials_nav2_utils_opencv_to_costmap.cpp
  src/tutorials_nav2_utils_dataset_loader.cpp
  src/tutorials_nav2_utils_planner_test.cpp
  src/tutorials_nav2_utils_controller_test.cpp)

# demo0 tutorials_nav2_demo0_path_planner_test.cpp
add_executable(tutorial.nav2.path_planner src/tutorials_nav2_demo0_path_planner_test ${SRCS})
ament_target_dependencies(tutorial.nav2.path_planner ${DEPS})

# demo1
add_executable(tutorial.nav2.b_spline_path_test src/tutorials_nav2_demo1_b_spline_path_test.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2.b_spline_path_test ${DEPS})

# demo2

# demo3
add_executable(tutorial.nav2.cubic_spline_path_test src/tutorials_nav2_demo3_cubic_spline_planner_test.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2.cubic_spline_path_test ${DEPS})

# demo4 tutorials_nav2_demo4_bezier_path_test.cpp
add_executable(tutorial.nav2.bezier_path src/tutorials_nav2_demo4_bezier_path_test.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2.bezier_path ${DEPS})

# demo5 tutorials_nav2_demo5_costmap_image_show_test.cpp
add_executable(tutorial.nav2.costmap_image_show src/tutorials_nav2_demo5_costmap_image_show_test ${SRCS})
ament_target_dependencies(tutorial.nav2.costmap_image_show ${DEPS})

# demo6 utorials_nav2_demo6_navfn_planner_test.cpp
add_executable(tutorial.nav2.navfn_planner src/tutorials_nav2_demo6_navfn_planner_test ${SRCS})
ament_target_dependencies(tutorial.nav2.navfn_planner ${DEPS})

# demo7 tutorials_nav2_demo7_a_star_planner_test.cpp
add_executable(tutorial.nav2.a_star_planner src/tutorials_nav2_demo7_a_star_planner_test ${SRCS})
ament_target_dependencies(tutorial.nav2.a_star_planner ${DEPS})

# demo8 tutorials_nav2_demo8_costmap_publish_test.cpp
add_executable(tutorial.nav2.costmap_publish_test src/tutorials_nav2_demo8_costmap_publish_test ${SRCS})
ament_target_dependencies(tutorial.nav2.costmap_publish_test ${DEPS})

# demo9 tutorials_nav2_demo9_pcl_filter_ground_test.cpp
# add_executable(tutorial.nav2.pcl_filter_ground src/tutorials_nav2_demo9_pcl_filter_ground_test ${SRCS})
# ament_target_dependencies(tutorial.nav2.pcl_filter_ground ${DEPS})

# tutorials_nav2_controller_node_main.cpp
add_executable(tutorial.nav2_controller_node src/tutorials_nav2_controller_node_main.cpp ${SRCS})
ament_target_dependencies(tutorial.nav2_controller_node ${DEPS})

install(TARGETS
  tutorial.nav2.b_spline_path_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tutorial.nav2.path_planner 
  tutorial.nav2.cubic_spline_path_test
  tutorial.nav2.bezier_path
  tutorial.nav2.costmap_image_show
  tutorial.nav2.navfn_planner
  tutorial.nav2.a_star_planner
  tutorial.nav2.costmap_publish_test
  # tutorial.nav2.pcl_filter_ground
  tutorial.nav2_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY images DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
