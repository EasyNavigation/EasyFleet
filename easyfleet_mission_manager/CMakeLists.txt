cmake_minimum_required(VERSION 3.20)
project(control_center)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(arch_mockup REQUIRED)
find_package(arch_mockup_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(perception_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/capability_discovery.cpp
  src/${PROJECT_NAME}/capability_info.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  arch_mockup::arch_mockup
  arch_mockup_interfaces::arch_mockup_interfaces
  nlohmann_json::nlohmann_json
)

add_executable(control_center_alone_node src/main_alone.cpp)
target_link_libraries(control_center_alone_node
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  nav2_msgs::nav2_msgs
  moveit_msgs::moveit_msgs
  trajectory_msgs::trajectory_msgs
  perception_interfaces::perception_interfaces
)

add_executable(control_center_collaboration_node src/main_collaboration.cpp)
target_link_libraries(control_center_collaboration_node
  ${PROJECT_NAME}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  nav2_msgs::nav2_msgs
  moveit_msgs::moveit_msgs
  trajectory_msgs::trajectory_msgs
  perception_interfaces::perception_interfaces
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
  TARGETS control_center_alone_node control_center_collaboration_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  arch_mockup
  arch_mockup_interfaces
  nlohmann_json
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
