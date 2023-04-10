cmake_minimum_required(VERSION 3.8)
project(spencer_tracking_rviz_plugin)

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
find_package(std_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(spencer_tracking_msgs REQUIRED)
find_package(spencer_human_attribute_msgs REQUIRED)
find_package(spencer_social_relation_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(visualization_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(DEPS 
  rclcpp
  std_msgs
  rviz2
  spencer_tracking_msgs
  spencer_human_attribute_msgs
  spencer_social_relation_msgs
  interactive_markers
  visualization_msgs)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

set(QT_LIBRARIES Qt5::Widgets)
  qt5_wrap_cpp(MOC_FILES
  src/detected_persons_display.h
  src/tracked_persons_display.h
  src/tracked_groups_display.h
  src/social_relations_display.h
  src/social_activities_display.h
  src/human_attributes_display.h
  src/person_display_common.h
  src/additional_topic_subscriber.h
)


add_definitions(-DQT_NO_KEYWORDS)

set(SRCS
  src/detected_persons_display.cpp
  src/tracked_persons_display.cpp
  src/tracked_groups_display.cpp
  src/social_relations_display.cpp
  src/social_activities_display.cpp
  src/human_attributes_display.cpp
  src/person_display_common.cpp
  src/tracked_persons_cache.cpp
  src/visuals/person_visual.cpp
  ${MOC_FILES}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS}) # for generation of message dependencies
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

install(PROGRAMS scripts/send_test_msgs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

ament_package()
