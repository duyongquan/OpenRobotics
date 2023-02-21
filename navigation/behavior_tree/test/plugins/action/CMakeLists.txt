
# test_back_up_action
ament_add_gtest(test_action_back_up_action test_back_up_action.cpp)
target_link_libraries(test_action_back_up_action back_up_action_bt_node)
ament_target_dependencies(test_action_back_up_action ${dependencies})

# test_navigate_to_pose_action
ament_add_gtest(test_action_navigate_to_pose_action test_navigate_to_pose_action.cpp)
target_link_libraries(test_action_navigate_to_pose_action navigate_to_pose_action_bt_node)
ament_target_dependencies(test_action_navigate_to_pose_action ${dependencies})

# # test_goal_reached
# ament_add_gtest(test_condition_goal_reached test_goal_reached.cpp)
# target_link_libraries(test_condition_goal_reached spin_action_bt_node)
# ament_target_dependencies(test_condition_goal_reached ${dependencies})

# install
install(TARGETS
    test_action_back_up_action
    test_action_navigate_to_pose_action
    DESTINATION lib/${PROJECT_NAME}
)

