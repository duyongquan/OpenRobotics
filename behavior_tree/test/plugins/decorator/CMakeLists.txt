
# test_decorator_rate_controller
ament_add_gtest(test_decorator_rate_controller test_rate_controller.cpp)
target_link_libraries(test_decorator_rate_controller rate_controller_bt_node)
ament_target_dependencies(test_decorator_rate_controller ${dependencies})

# test_speed_controller
ament_add_gtest(test_decorator_speed_controller test_speed_controller.cpp)
target_link_libraries(test_decorator_speed_controller speed_controller_bt_node)
ament_target_dependencies(test_decorator_speed_controller ${dependencies})

# install
install(TARGETS
    test_decorator_rate_controller
    test_decorator_speed_controller
    DESTINATION lib/${PROJECT_NAME}
)
