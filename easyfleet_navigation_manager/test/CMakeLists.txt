find_package(ament_cmake_gtest REQUIRED)

# Pure logic, no ROS node/topics involved -- no domain isolation needed.
ament_add_gtest(conflict_detector_tests conflict_detector_tests.cpp)
target_link_libraries(conflict_detector_tests
  ${PROJECT_NAME}
  GTest::gtest
)

# Fabricates traffic on the *global* topics /global_map and
# /global_routes (this package's whole point is that they're
# fleet-wide, not per-robot) -- domain-isolate so this test binary
# can't cross-talk with any other test or a live navigation_manager_node.
ament_add_gtest(test_navigation_manager test_navigation_manager.cpp
  TIMEOUT 60
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=95)
target_link_libraries(test_navigation_manager
  ${PROJECT_NAME}
  GTest::gtest
)
