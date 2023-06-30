
# test_recovery_node
ament_add_gtest(test_control_recovery_node test_recovery_node.cpp)
target_link_libraries(test_control_recovery_node recovery_node_bt_node)
ament_target_dependencies(test_control_recovery_node ${dependencies})

# test_pipeline_sequence
ament_add_gtest(test_control_pipeline_sequence test_pipeline_sequence.cpp)
target_link_libraries(test_control_pipeline_sequence pipeline_sequence_bt_node)
ament_target_dependencies(test_control_pipeline_sequence ${dependencies})

# install
install(TARGETS
    test_control_recovery_node
    test_control_pipeline_sequence
    DESTINATION lib/${PROJECT_NAME}
)

