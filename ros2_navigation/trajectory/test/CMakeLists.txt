# Test test_traj_generator
add_executable(test_traj_generator test_traj_generator.cpp)
ament_target_dependencies(test_traj_generator ${dependencies})
target_link_libraries(test_traj_generator ${library_name})
