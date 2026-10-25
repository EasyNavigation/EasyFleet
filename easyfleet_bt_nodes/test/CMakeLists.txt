find_package(ament_cmake_gtest REQUIRED)

# Own domain, localhost only: the fake capability servers use fixed action names.
ament_add_gtest(test_capability_nodes test_capability_nodes.cpp
  TIMEOUT 120 ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=71)
target_compile_definitions(test_capability_nodes PRIVATE
  EASYFLEET_BT_NODES_PLUGIN="$<TARGET_FILE:easyfleet_capability_bt_nodes>")
add_dependencies(test_capability_nodes easyfleet_capability_bt_nodes)
target_link_libraries(test_capability_nodes
  behaviortree_cpp::behaviortree_cpp
  ${easyfleet_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)
