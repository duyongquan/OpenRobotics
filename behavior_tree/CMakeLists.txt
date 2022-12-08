cmake_minimum_required(VERSION 3.8)
project(behavior_tree)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  nav2_util
  nav2_msgs
  behaviortree_cpp_v3)

# sources
# set(SRCS 
#   src/tutorials_topic_demo1_initial_env.cpp)

############################### sample_nodes #################################
include_directories(src/sample_nodes)
set(SAMPLE_SRCS
  src/sample_nodes/crossdoor_nodes.cpp
  src/sample_nodes/dummy_nodes.cpp
  src/sample_nodes/movebase_node.cpp)

add_library(bt_sample_nodes STATIC ${SAMPLE_SRCS})
list(APPEND plugin_libs bt_sample_nodes)

add_library(crossdoor_nodes_dyn SHARED src/sample_nodes/crossdoor_nodes.cpp)
list(APPEND plugin_libs crossdoor_nodes_dyn)

add_library(dummy_nodes_dyn SHARED src/sample_nodes/dummy_nodes.cpp)
list(APPEND plugin_libs dummy_nodes_dyn)

add_library(movebase_node_dyn SHARED src/sample_nodes/movebase_node.cpp)
list(APPEND plugin_libs movebase_node_dyn)
############################### sample_nodes #################################

# tutorials_01_build_your_first_tree_static 
add_executable(tutorials_01_build_your_first_tree_static  src/tutorials_01_build_your_first_tree.cpp )
target_compile_definitions(tutorials_01_build_your_first_tree_static PRIVATE "MANUAL_STATIC_LINKING")
target_link_libraries(tutorials_01_build_your_first_tree_static bt_sample_nodes)

# tutorials_01_build_your_first_tree_static shared
add_executable(tutorials_01_first_tree_dynamic  src/tutorials_01_build_your_first_tree.cpp)
target_link_libraries(tutorials_01_first_tree_dynamic bt_sample_nodes)

# tutorials_02_basic_ports
add_executable(tutorials_02_basic_ports  src/tutorials_02_basic_ports.cpp)
target_link_libraries(tutorials_02_basic_ports bt_sample_nodes)

# tutorials_03_generic_ports
add_executable(tutorials_03_generic_ports  src/tutorials_03_generic_ports.cpp)
target_link_libraries(tutorials_03_generic_ports bt_sample_nodes)

# tutorials_04_reactive_sequence
add_executable(tutorials_04_reactive_sequence  src/tutorials_04_reactive_sequence.cpp)
target_link_libraries(tutorials_04_reactive_sequence bt_sample_nodes)

# tutorials_05_crossdoor
add_executable(tutorials_05_crossdoor  src/tutorials_05_crossdoor.cpp)
target_link_libraries(tutorials_05_crossdoor bt_sample_nodes)

# tutorials_06_subtree_port_remapping
add_executable(tutorials_06_subtree_port_remapping  src/tutorials_06_subtree_port_remapping.cpp)
target_link_libraries(tutorials_06_subtree_port_remapping bt_sample_nodes)

# tutorials_07_load_multiple_xml
add_executable(tutorials_07_load_multiple_xml  src/tutorials_07_load_multiple_xml.cpp)
target_link_libraries(tutorials_07_load_multiple_xml bt_sample_nodes)

# tutorials_08_additional_node_args
add_executable(tutorials_08_additional_node_args  src/tutorials_08_additional_node_args.cpp)
target_link_libraries(tutorials_08_additional_node_args bt_sample_nodes)

# tutorials_09_async_actions_coroutines
add_executable(tutorials_09_async_actions_coroutines  src/tutorials_09_async_actions_coroutines.cpp)
target_link_libraries(tutorials_09_async_actions_coroutines bt_sample_nodes)

# tutorials_10_wrap_legacy
add_executable(tutorials_10_wrap_legacy src/tutorials_10_wrap_legacy.cpp)
target_link_libraries(tutorials_10_wrap_legacy bt_sample_nodes)

# tutorials_11_runtime_ports
add_executable(tutorials_11_runtime_ports src/tutorials_11_runtime_ports.cpp)
target_link_libraries(tutorials_11_runtime_ports bt_sample_nodes)

# tutorials_12_ncurses_manual_selector
add_executable(tutorials_12_ncurses_manual_selector src/tutorials_12_ncurses_manual_selector.cpp)
target_link_libraries(tutorials_12_ncurses_manual_selector bt_sample_nodes)

# tutorials_13_waypoints
add_executable(tutorials_13_waypoints src/tutorials_13_waypoints.cpp)
target_link_libraries(tutorials_13_waypoints bt_sample_nodes)

# tutorials_14_broken_sequence
# add_executable(tutorials_14_broken_sequence src/tutorials_14_broken_sequence.cpp)
# target_link_libraries(tutorials_14_broken_sequence bt_sample_nodes)

# spin_action_bt_node
add_library(spin_action_bt_node SHARED plugins/action/spin_action.cpp)
list(APPEND plugin_libs spin_action_bt_node)

# navigate_to_pose_action_bt_node
add_library(navigate_to_pose_action_bt_node SHARED plugins/action/navigate_to_pose_action.cpp)
list(APPEND plugin_libs navigate_to_pose_action_bt_node)

# back_up_action_bt_node
add_library(back_up_action_bt_node SHARED plugins/action/back_up_action.cpp)
list(APPEND plugin_libs back_up_action_bt_node)

# goal_reached_condition_bt_node
add_library(goal_reached_condition_bt_node SHARED plugins/condition/goal_reached_condition.cpp)
list(APPEND plugin_libs goal_reached_condition_bt_node)

# is_battery_low_bt_node
add_library(is_battery_low_bt_node SHARED plugins/condition/is_battery_low_condition.cpp)
list(APPEND plugin_libs is_battery_low_bt_node)

# is_stuck_condition_bt_node
add_library(is_stuck_bt_node SHARED plugins/condition/is_stuck_condition.cpp)
list(APPEND plugin_libs is_stuck_bt_node)

# pipeline_sequence_bt_node
add_library(pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs pipeline_sequence_bt_node)

# recovery_node_bt_node
add_library(recovery_node_bt_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node_bt_node)

# rate_controller_bt_node
add_library(rate_controller_bt_node SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs rate_controller_bt_node)

# speed_controller
add_library(speed_controller_bt_node SHARED plugins/decorator/speed_controller.cpp)
list(APPEND plugin_libs speed_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  tutorials_01_build_your_first_tree_static
  tutorials_01_first_tree_dynamic
  tutorials_02_basic_ports
  tutorials_03_generic_ports
  tutorials_04_reactive_sequence
  tutorials_05_crossdoor
  tutorials_06_subtree_port_remapping
  tutorials_07_load_multiple_xml
  tutorials_08_additional_node_args
  tutorials_09_async_actions_coroutines
  tutorials_10_wrap_legacy
  tutorials_11_runtime_ports
  tutorials_12_ncurses_manual_selector
  tutorials_13_waypoints
  # tutorials_14_broken_sequence
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_dependencies(${dependencies})
ament_package()
