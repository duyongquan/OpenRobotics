cmake_minimum_required(VERSION 3.8)
project(fast_methods)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include
include_directories(
  include
)

# dependency
set(dependencies 
  rclcpp
  std_msgs)

# sources
set(SRCS 
  src/console/console.cpp
  src/fm/fmdata/fmcell.cpp
  src/ndgridmap/cell.cpp)

# headers
set(HDRS 
  fast_methods/console/console.h
  datastructures/fmcompare.hpp
  datastructures/fmdaryheap.hpp
  datastructures/fmfibheap.hpp
  datastructures/fmpriorityqueue.hpp
  datastructures/fmuntidyqueue.hpp
  fm/fmdata/fmcell.h
  fm/fmdata/fmcompare.hpp
  fm/fmdata/fmdaryheap.hpp
  fm/fmdata/fmfibheap.hpp
  fm/fmdata/fmpriorityqueue.hpp
  fm/fmdata/fmuntidyqueue.hpp
  fm/ddqm.hpp
  fm/eikonalsolver.hpp
  fm/fim.hpp
  fm/fmm.hpp
  fm/fmmstar.hpp
  fm/fsm.hpp
  fm/gmm.hpp
  fm/lsm.hpp
  fm/sfmm.hpp
  fm/sfmmstar.hpp
  fm/solver.hpp
  fm/ufmm.hpp
  fm2/fm2.hpp
  fm2/fm2star.hpp
  gradientdescent/gradientdescent.hpp
  io/gridplotter.hpp
  io/gridpoints.hpp
  io/gridwriter.hpp
  io/maploader.hpp
  ndgridmap/cell.h
  ndgridmap/fmcell.h
  ndgridmap/ndgridmap.h
  utils/utils.h)
  
# thirdparty
set(EXTRA_HDRS
  thirdparty/CImg.h
  thirdparty/fast_marching.hpp
  thirdparty/queue.hpp
  thirdparty/untidy_queue.hpp)

set(library_name fast_methods)

add_library(${library_name} SHARED ${SRCS})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES data DESTINATION share/${PROJECT_NAME})
install(FILES doc DESTINATION share/${PROJECT_NAME})


# map_generator
add_executable(ros2_navigation.fast_methods src/main.cpp ${SRCS})
ament_target_dependencies(ros2_navigation.fast_methods ${dependencies})

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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
