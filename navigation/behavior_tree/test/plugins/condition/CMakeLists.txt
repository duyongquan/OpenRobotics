
# test_is_battery_low
ament_add_gtest(test_condition_is_battery_low test_is_battery_low.cpp)
target_link_libraries(test_condition_is_battery_low is_battery_low_bt_node)
ament_target_dependencies(test_condition_is_battery_low ${dependencies})

# test_is_stuck
ament_add_gtest(test_condition_is_stuck test_is_stuck.cpp)
target_link_libraries(test_condition_is_stuck is_stuck_bt_node)
ament_target_dependencies(test_condition_is_stuck ${dependencies})

# test_goal_reached
ament_add_gtest(test_condition_goal_reached test_goal_reached.cpp)
target_link_libraries(test_condition_goal_reached goal_reached_condition_bt_node)
ament_target_dependencies(test_condition_goal_reached ${dependencies})

# install
install(TARGETS
    test_condition_is_battery_low
    test_condition_is_stuck
    test_condition_goal_reached
    DESTINATION lib/${PROJECT_NAME}
)

