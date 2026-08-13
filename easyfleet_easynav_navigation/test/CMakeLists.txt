find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)

# Confine DDS discovery to localhost and give this test binary its own
# ROS_DOMAIN_ID: it exercises easynav::GoalManagerClient, which talks on
# the shared/global easynav_control topic, so without domain isolation it
# could cross-talk with any other test binary or capability node using
# that same topic.
ament_add_gtest(test_bt_nodes test_bt_nodes.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=81)
target_link_libraries(test_bt_nodes
  easynav_navigate_bt_node
  rclcpp::rclcpp
  GTest::gtest
)
