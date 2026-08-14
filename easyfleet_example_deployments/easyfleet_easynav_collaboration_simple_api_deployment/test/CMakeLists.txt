find_package(ament_cmake_gtest REQUIRED)
find_package(geometry_msgs REQUIRED)

# Confine DDS discovery to localhost (subnet-wide traffic is noticeably
# slower and, on loaded/shared machines, occasionally flaky) and give each
# test binary its own ROS_DOMAIN_ID: these tests exercise
# easyfleet_core::Capability, which publishes on the *global* topics
# /capabilities and /capabilities_status, so without domain isolation they
# could cross-talk with any other test binary or capability node using
# those same topics. IDs 175-178 are used here specifically to stay clear
# of easyfleet_easynav_collaboration_deployment's own 171-174: this
# package started as a straight copy of that one (see refactor.md), so its
# test suite originally carried the exact same IDs -- harmless standalone,
# but colcon can run both packages' test binaries concurrently, and same
# domain ID across two *different* packages is exactly the cross-talk this
# comment already warned about, just not for the sibling pointed to here.
ament_add_gtest(test_perception_fake_action_server test_perception_fake_action_server.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=175)
target_link_libraries(test_perception_fake_action_server
  ${PROJECT_NAME}
  GTest::gtest
)

ament_add_gtest(test_perception_fake_capability test_perception_fake_capability.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=176)
target_link_libraries(test_perception_fake_capability
  ${PROJECT_NAME}
  GTest::gtest
)

ament_add_gtest(test_manipulation_fake_action_server test_manipulation_fake_action_server.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=177)
target_link_libraries(test_manipulation_fake_action_server
  ${PROJECT_NAME}
  GTest::gtest
)

ament_add_gtest(test_manipulation_fake_capability test_manipulation_fake_capability.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=178)
target_link_libraries(test_manipulation_fake_capability
  ${PROJECT_NAME}
  GTest::gtest
)
