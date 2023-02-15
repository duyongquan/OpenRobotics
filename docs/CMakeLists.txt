cmake_minimum_required (VERSION 2.8.7)

project(ros2_tutorials)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(PythonInterp REQUIRED)
find_package(Sphinx REQUIRED)

if(SPHINX_FOUND)
  set(OUTPUT_DIR "${PROJECT_BINARY_DIR}/docs/html")
  add_custom_target(build_doc ALL
      ${SPHINX_EXECUTABLE} -b html
      ${CMAKE_CURRENT_SOURCE_DIR}/source
      ${CMAKE_CURRENT_BINARY_DIR}/html
      COMMENT "Building documentation."
  )
endif()