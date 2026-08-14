find_package(ament_cmake_gtest REQUIRED)
find_package(example_interfaces REQUIRED)

# Test-only CapabilityFactory plugin (test_capability_factory_plugin.cpp),
# registered exactly like a real deployment package's own plugin, so
# test_deployment.cpp can exercise Deployment::add_capability()'s success
# path -- not just the "unknown plugin" failure path -- without a
# dependency on any downstream deployment package. Only ever built/exported
# under BUILD_TESTING (this whole directory is only added then), and its
# lookup name ("test_capability/...") can't collide with a real capability.
add_library(test_capability_factory_plugin SHARED test_capability_factory_plugin.cpp)
target_link_libraries(test_capability_factory_plugin
  ${PROJECT_NAME}
  pluginlib::pluginlib
  example_interfaces::example_interfaces
)
install(TARGETS test_capability_factory_plugin DESTINATION lib)
# NOTE: pluginlib_export_plugin_description_file() itself is called from the
# top-level CMakeLists.txt, not here -- it's a macro that accumulates into a
# variable consumed by ament_package(), and add_subdirectory() gives this
# file its own CMake variable scope that a top-level ament_package() call
# can't see into.

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

ament_add_gtest(test_deployment test_deployment.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=66)
target_link_libraries(test_deployment
  ${PROJECT_NAME}
  GTest::gtest
)
add_dependencies(test_deployment test_capability_factory_plugin)

# Isolated from test_deployment.cpp/every other test binary: Deployment::
# shutdown() calls rclcpp::shutdown(), which invalidates the ROS context
# for the rest of *this process* -- fine as this binary's one and only
# test, but it would break every test running after it if it shared a
# binary (and therefore a process, and therefore an rclcpp context) with
# them.
ament_add_gtest(test_deployment_shutdown test_deployment_shutdown.cpp
  TIMEOUT 60 SKIP_LINKING_MAIN_LIBRARIES
  ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_DOMAIN_ID=67)
target_link_libraries(test_deployment_shutdown
  ${PROJECT_NAME}
  GTest::gtest
)
add_dependencies(test_deployment_shutdown test_capability_factory_plugin)
