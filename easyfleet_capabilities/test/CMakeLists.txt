find_package(ament_cmake_gtest REQUIRED)

# Confine DDS discovery to localhost (subnet-wide traffic is noticeably
# slower and, on loaded/shared machines, occasionally flaky) and give each
# test binary its own ROS_DOMAIN_ID: these tests exercise
# arch_mockup::Capability, which publishes on the *global* topics
# /capabilities and /capabilities_status, so without domain isolation they
# could cross-talk with any other test binary or capability node using
# those same topics.
ament_add_gtest(test_execute_trajectory_action_server test_execute_trajectory_action_server.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=71)
target_link_libraries(test_execute_trajectory_action_server
  ${PROJECT_NAME}
  GTest::gtest
)

ament_add_gtest(test_manipulation_capability test_manipulation_capability.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=72)
target_link_libraries(test_manipulation_capability
  ${PROJECT_NAME}
  GTest::gtest
)
