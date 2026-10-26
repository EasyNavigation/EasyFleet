cmake_minimum_required(VERSION 3.20)
project(easyfleet_core)

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
find_package(action_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(easyfleet_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/deployment.cpp
  src/init.cpp
  src/link_watchdog.cpp
  src/order_history.cpp
  src/param_utils.cpp
  src/robot_status_publisher.cpp
  src/spin_utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  action_msgs::action_msgs
  rcl_interfaces::rcl_interfaces
  lifecycle_msgs::lifecycle_msgs
  easyfleet_interfaces::easyfleet_interfaces
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pluginlib::pluginlib
  ament_index_cpp::ament_index_cpp
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  action_msgs
  rcl_interfaces
  lifecycle_msgs
  easyfleet_interfaces
  geometry_msgs
  sensor_msgs
  pluginlib
  ament_index_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(pluginlib REQUIRED)

  # Must be called from this top-level scope, not from test/CMakeLists.txt:
  # it's a macro that accumulates into a variable consumed by the
  # ament_package() call below, and add_subdirectory(test) would give that
  # accumulation its own CMake variable scope invisible from here.
  pluginlib_export_plugin_description_file(easyfleet_core test/test_plugins.xml)

  add_subdirectory(test)
endif()

ament_package()
