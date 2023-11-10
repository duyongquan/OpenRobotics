cmake_minimum_required(VERSION 3.8)
project(casadi_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include("${PROJECT_SOURCE_DIR}/cmake/FindCASADI.cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(CASADI REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${CASADI_INCLUDE_DIR})


message("CASADI_LIBRARIES: ${CASADI_LIBRARIES}")

# tutorials01_test_casadi_env
add_executable(tutorials01_test_casadi_env src/tutorials01_test_casadi_env.cpp)
target_link_libraries(tutorials01_test_casadi_env ${CASADI_LIBRARIES})

# tutorials02_test_casadi_rosenbrock
add_executable(tutorials02_test_casadi_rosenbrock src/tutorials02_test_casadi_rosenbrock.cpp)
target_link_libraries(tutorials02_test_casadi_rosenbrock ${CASADI_LIBRARIES})

install(TARGETS 
  tutorials01_test_casadi_env
  tutorials02_test_casadi_rosenbrock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  tutorials01_test_casadi_env
  tutorials02_test_casadi_rosenbrock
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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
