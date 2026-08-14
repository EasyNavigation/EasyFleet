find_package(ament_cmake_gtest REQUIRED)

# Confine DDS discovery to localhost (subnet-wide traffic is noticeably
# slower and, on loaded/shared machines, occasionally flaky) and give this
# test binary its own ROS_DOMAIN_ID: it deliberately fabricates traffic on
# the *global* topics /capabilities and /capabilities_status, so without
# domain isolation it could cross-talk with any other test binary or
# capability node using those same topics.
ament_add_gtest(test_capability_discovery test_capability_discovery.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=70)
target_link_libraries(test_capability_discovery
  ${PROJECT_NAME}
  GTest::gtest
)

# TEMPORARY: syntax/type-check only for the not-yet-implemented
# RobotHandle/SimpleController headers -- remove once they have real
# bodies + real tests.
ament_add_gtest(scratch_syntax_check scratch_syntax_check.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(scratch_syntax_check
  ${PROJECT_NAME}
  GTest::gtest
)
