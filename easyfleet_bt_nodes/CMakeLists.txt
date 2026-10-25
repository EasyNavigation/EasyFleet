cmake_minimum_required(VERSION 3.20)
project(easyfleet_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(easyfleet_core REQUIRED)
find_package(easyfleet_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(easyfleet_capability_bt_nodes SHARED
  src/${PROJECT_NAME}/capability_nodes.cpp
)
target_include_directories(easyfleet_capability_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(easyfleet_capability_bt_nodes PUBLIC
  behaviortree_cpp::behaviortree_cpp
  easyfleet_core::easyfleet_core
  ${easyfleet_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  PRIVATE
  nlohmann_json::nlohmann_json
)
target_compile_definitions(easyfleet_capability_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS easyfleet_capability_bt_nodes
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  behaviortree_cpp
  easyfleet_core
  easyfleet_interfaces
  rclcpp
  rclcpp_action
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

ament_package()
