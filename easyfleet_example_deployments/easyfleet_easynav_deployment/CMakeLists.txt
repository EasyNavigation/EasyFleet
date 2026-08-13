cmake_minimum_required(VERSION 3.20)
project(easyfleet_easynav_deployment)

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
find_package(easyfleet_mission_manager REQUIRED)
find_package(easynav_system REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(easynav_mission_node src/main_easynav.cpp)
target_link_libraries(easynav_mission_node
  rclcpp::rclcpp
  easyfleet_core::easyfleet_core
  easyfleet_mission_manager::easyfleet_mission_manager
)

# The navigation capability itself: this deployment's own
# easyfleet_core::NavigationActionServerBase subclass, driving EasyNav
# through easynav::GoalManagerClient and ticking a BehaviorTree.CPP tree
# per goal. Deliberately owned here rather than in
# easyfleet_easynav_navigation: how a capability turns waypoints/BT plugins
# into a running ROS action is a manufacturer/integrator choice, not
# something EasyFleet prescribes -- that package only provides the reusable
# Navigate BT node this class loads as a plugin (see bt_plugins below and
# easynav_navigation_capability.hpp).
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/easynav_navigation_capability.cpp
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
  easynav_system::easynav_system
  behaviortree_cpp::behaviortree_cpp
  geometry_msgs::geometry_msgs
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  nlohmann_json::nlohmann_json
)

add_executable(easynav_navigation_capability_node src/easynav_navigation_capability_main.cpp)
target_link_libraries(easynav_navigation_capability_node
  ${PROJECT_NAME}
  rclcpp::rclcpp
  lifecycle_msgs::lifecycle_msgs
)

# StartOff/Finish: the fake bookend BT nodes for this scenario's
# navigate.xml tree, each built as its own BT.CPP plugin (BT_REGISTER_NODES,
# loaded by path via the navigation capability's `bt_plugins` parameter --
# see launch/easynav_robot_launch.yaml). Kept here, not in
# easyfleet_easynav_navigation, since they are fake/deployment-specific,
# unlike that package's real Navigate node.
add_library(start_off_bt_node SHARED
  src/${PROJECT_NAME}/start_off.cpp
)
target_include_directories(start_off_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(start_off_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
)
target_compile_definitions(start_off_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(finish_bt_node SHARED
  src/${PROJECT_NAME}/finish.cpp
)
target_include_directories(finish_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(finish_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
)
target_compile_definitions(finish_bt_node PRIVATE BT_PLUGIN_EXPORT)

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
  TARGETS easynav_mission_node easynav_navigation_capability_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS start_off_bt_node finish_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  easyfleet_core
  easynav_system
  behaviortree_cpp
  geometry_msgs
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

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
