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

# Pure logic, no ROS graph involved -- no domain isolation needed.
ament_add_gtest(test_throttle test_throttle.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_throttle
  ${PROJECT_NAME}
  GTest::gtest
)

# Pure logic over message structs, no ROS graph involved -- no domain
# isolation needed.
ament_add_gtest(test_mission_helpers test_mission_helpers.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES)
target_link_libraries(test_mission_helpers
  ${PROJECT_NAME}
  GTest::gtest
)

# Brings up real (fake-backed) capability nodes and drives them through
# FleetSession/RobotHandle/SimpleController end to end -- fabricates
# traffic on /capabilities, /capabilities_status and the capability's own
# action, same domain-isolation reasoning as test_capability_discovery
# above.
ament_add_gtest(test_fleet_session_robot_handle test_fleet_session_robot_handle.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=90)
target_link_libraries(test_fleet_session_robot_handle
  ${PROJECT_NAME}
  GTest::gtest
)

# Isolated from every other FleetSession/RobotHandle test: FleetSession::
# shutdown() calls rclcpp::shutdown(), which would break every test
# running after it in the same process.
ament_add_gtest(test_fleet_session_shutdown test_fleet_session_shutdown.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=91)
target_link_libraries(test_fleet_session_shutdown
  ${PROJECT_NAME}
  GTest::gtest
)
