cmake_minimum_required(VERSION 3.20)
project(easyfleet_easynav_collaboration_simple_api_deployment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(easyfleet_core REQUIRED)
find_package(easyfleet_interfaces REQUIRED)
find_package(easyfleet_mission_manager REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)

# perception + manipulation only: this deployment's navigation capability is
# real EasyNav, reused directly from easyfleet_easynav_deployment's own
# easynav_navigation_capability_node executable (see launch/robot_*_launch.yaml)
# -- no navigation_fake_capability here, unlike the sibling fake deployments.
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/perception_fake_capability.cpp
  src/${PROJECT_NAME}/manipulation_fake_capability.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  easyfleet_core::easyfleet_core
  easyfleet_interfaces::easyfleet_interfaces
  vision_msgs::vision_msgs
  ${builtin_interfaces_TARGETS}
  pluginlib::pluginlib
)
# Makes perception and manipulation loadable by pluginlib lookup name (see
# easyfleet_core::Deployment::add_capability(), used by robot_node.cpp) --
# registered in perception_fake_capability.cpp/manipulation_fake_capability.cpp
# via PLUGINLIB_EXPORT_CLASS.
pluginlib_export_plugin_description_file(
  easyfleet_core easyfleet_easynav_collaboration_simple_api_deployment_plugins.xml)

# The same robot as easyfleet_easynav_collaboration_deployment's own
# robot_node.cpp, driven through easyfleet_core::Deployment's simpler API
# instead of that package's hand-rolled lifecycle dance -- see refactor.md.
# Produces identical ROS-level behavior (same capability/action names), so
# a launch file can point at either package interchangeably.
add_executable(robot_node src/robot_node.cpp)
target_link_libraries(robot_node
  ${PROJECT_NAME}
  easyfleet_core::easyfleet_core
)

# The simple-API mission script (easyfleet_mission_manager::RobotHandle/
# SimpleController/FleetSession) -- see refactor.md for the implementation
# history; produces identical mission behavior to
# easyfleet_easynav_collaboration_deployment's own
# main_easynav_collaboration.cpp, driven through the simpler API instead.
add_executable(easynav_collaboration_mission_node src/main_easynav_collaboration.cpp)
target_link_libraries(easynav_collaboration_mission_node
  rclcpp::rclcpp
  easyfleet_core::easyfleet_core
  easyfleet_mission_manager::easyfleet_mission_manager
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS
    robot_node
    easynav_collaboration_mission_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  easyfleet_core
  easyfleet_interfaces
  easyfleet_mission_manager
  vision_msgs
  builtin_interfaces
  pluginlib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
