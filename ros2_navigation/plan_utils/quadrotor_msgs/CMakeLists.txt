cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED) 

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
  ADD_LINTER_TESTS 
)

install(DIRECTORY include/
  DESTINATION ${PROJECT_NAME}/
)

set(dependencies
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs)

# add_executable(test_exe src/test_exe.cpp)
add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

ament_target_dependencies(decode_msgs ${dependencies})
ament_target_dependencies(encode_msgs ${dependencies})

install(TARGETS decode_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin

)

install(TARGETS encode_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(decode_msgs)
ament_export_libraries(encode_msgs)
ament_export_dependencies(${dependencies})
ament_package()
