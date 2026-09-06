# Test dynamic parameters
ament_add_gtest(test_controller_server
  test_controller_server.cpp
)
ament_target_dependencies(test_controller_server
  ${dependencies}
)
target_link_libraries(test_controller_server
  ${library_name}
)

ament_add_gtest(test_control_loop_rate test_control_loop_rate.cpp)
ament_target_dependencies(test_control_loop_rate rclcpp)
target_include_directories(test_control_loop_rate PRIVATE ../include)
