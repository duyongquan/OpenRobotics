# tests for regulated PP
ament_add_gtest(linear_interpolation_test
  linear_interpolation_test.cpp
)

ament_target_dependencies(linear_interpolation_test
  ${dependencies}
)
target_link_libraries(linear_interpolation_test
  ${library_name}
)
