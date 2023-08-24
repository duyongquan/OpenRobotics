cmake_minimum_required(VERSION 3.8)
project(decomp_ros_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(decomp_ros_msgs REQUIRED) 
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${DECOMP_UTIL_INCLUDE_DIRS})

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  std_msgs
  nav_msgs
  nav2_msgs
  pluginlib
  Qt5
  geometry_msgs
  decomp_ros_msgs
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
  tf2
  tf2_geometry_msgs
  visualization_msgs
  )

set(nav2_rviz_plugins_headers_to_moc
  include/decomp_ros_utils/bound_visual.hpp
  # include/decomp_ros_utils/data_ros_utils.hpp
  # include/decomp_ros_utils/ellipsoid_array_display.hpp
  # include/decomp_ros_utils/ellipsoid_array_visual.hpp
  # include/decomp_ros_utils/mesh_visual.hpp
  # include/decomp_ros_utils/polyhedron_array_display.hpp
  # include/decomp_ros_utils/vector_visual.hpp
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/bound_visual.cpp
  src/ellipsoid_array_display.cpp
  src/ellipsoid_array_visual.cpp
  src/mesh_visual.cpp
  src/polyhedron_array_display.cpp
  src/vector_visual.cpp
  ${nav2_rviz_plugins_headers_to_moc}
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
