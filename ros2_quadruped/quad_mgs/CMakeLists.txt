cmake_minimum_required(VERSION 3.8)
project(quad_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/BodyPlan.msg"
  # "msg/BodyState.msg"
  # "msg/ContactMode.msg"
  # "msg/FootPlanDiscrete.msg"
  # "msg/FootState.msg"
  # "msg/GRFArray.msg"
  # "msg/LegCommand.msg"
  # "msg/LegCommandArray.msg"
  # "msg/LegContactMode.msg"
  # "msg/LocalPlan.msg"
  # "msg/MotorCommand.msg"
  # "msg/MultiFootPlanContinuous.msg"
  # "msg/MultiFootPlanDiscrete.msg"
  # "msg/MultiFootState.msg"
  # "msg/RobotPlan.msg"
  # "msg/RobotPlanDiagnostics.msg"
  # "msg/RobotState.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()
