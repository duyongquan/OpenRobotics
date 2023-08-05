cmake_minimum_required(VERSION 3.8)
project(quad_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BodyPlan.msg"
  "msg/BodyState.msg"
  "msg/ContactMode.msg"
  "msg/FootPlanDiscrete.msg"
  "msg/FootState.msg"
  "msg/GRFArray.msg"
  "msg/LegCommand.msg"
  "msg/LegCommandArray.msg"
  "msg/LegContactMode.msg"
  "msg/LocalPlan.msg"
  "msg/MotorCommand.msg"
  "msg/MultiFootPlanContinuous.msg"
  "msg/MultiFootPlanDiscrete.msg"
  "msg/MultiFootState.msg"
  "msg/RobotPlan.msg"
  "msg/RobotPlanDiagnostics.msg"
  "msg/RobotState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs action_msgs nav_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
