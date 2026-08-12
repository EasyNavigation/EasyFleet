find_package(ament_cmake_gtest REQUIRED)
find_package(example_interfaces REQUIRED)

ament_add_gtest(test_param_utils test_param_utils.cpp)
target_link_libraries(test_param_utils ${PROJECT_NAME})

# Confine DDS discovery to localhost (subnet-wide traffic is noticeably
# slower and, on loaded/shared machines, occasionally flaky) and give each
# test binary its own ROS_DOMAIN_ID. The latter matters because several of
# these tests exercise easyfleet_core::Capability, which publishes on the
# *global* topics /capabilities and /capabilities_status: without domain
# isolation, two test binaries running concurrently (colcon test's default),
# or even an unrelated capability node someone happens to have running
# interactively, can cross-talk on those topics and fail non-deterministically.
ament_add_gtest(test_action_server_base test_action_server_base.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=61)
target_link_libraries(test_action_server_base
  ${PROJECT_NAME}
  example_interfaces::example_interfaces
  GTest::gtest
)

ament_add_gtest(test_action_client test_action_client.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=62)
target_link_libraries(test_action_client
  ${PROJECT_NAME}
  example_interfaces::example_interfaces
  GTest::gtest
)

ament_add_gtest(test_integration test_integration.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=63)
target_link_libraries(test_integration
  ${PROJECT_NAME}
  example_interfaces::example_interfaces
  GTest::gtest
)

ament_add_gtest(test_capability test_capability.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=64)
target_link_libraries(test_capability
  ${PROJECT_NAME}
  example_interfaces::example_interfaces
  GTest::gtest
)

ament_add_gtest(test_capability_client test_capability_client.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=65)
target_link_libraries(test_capability_client
  ${PROJECT_NAME}
  example_interfaces::example_interfaces
  GTest::gtest
)
