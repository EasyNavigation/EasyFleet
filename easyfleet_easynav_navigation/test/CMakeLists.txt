find_package(ament_cmake_gtest REQUIRED)

# Confine DDS discovery to localhost and give each test binary its own
# ROS_DOMAIN_ID: these tests exercise easynav::GoalManagerClient and
# easyfleet_core::Capability, both of which talk on shared/global topics
# (easynav_control, /capabilities, /capabilities_status), so without domain
# isolation they could cross-talk with any other test binary or capability
# node using those same topics.
ament_add_gtest(test_bt_nodes test_bt_nodes.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=81)
target_link_libraries(test_bt_nodes
  ${PROJECT_NAME}
  GTest::gtest
)

ament_add_gtest(test_easynav_navigation_capability test_easynav_navigation_capability.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=82)
target_link_libraries(test_easynav_navigation_capability
  ${PROJECT_NAME}
  GTest::gtest
)
