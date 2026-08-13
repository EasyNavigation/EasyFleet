find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_bt_nodes test_bt_nodes.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_bt_nodes
  start_off_bt_node
  finish_bt_node
  GTest::gtest
)

# Confine DDS discovery to localhost and give this test binary its own
# ROS_DOMAIN_ID: it exercises easyfleet_core::Capability, which publishes
# on the *global* topics /capabilities and /capabilities_status, so
# without domain isolation it could cross-talk with any other test binary
# or capability node using those same topics.
ament_add_gtest(test_easynav_navigation_capability test_easynav_navigation_capability.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=83)
target_link_libraries(test_easynav_navigation_capability
  ${PROJECT_NAME}
  GTest::gtest
)
