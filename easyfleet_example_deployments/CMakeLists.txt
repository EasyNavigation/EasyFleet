cmake_minimum_required(VERSION 3.20)
project(easyfleet_example_deployments)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/navigation_fake_capability.cpp
  src/${PROJECT_NAME}/manipulation_fake_capability.cpp
  src/${PROJECT_NAME}/perception_fake_capability.cpp
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
  geometry_msgs::geometry_msgs
  sensor_msgs::sensor_msgs
  vision_msgs::vision_msgs
)

foreach(capability navigation manipulation perception)
  add_executable(${capability}_fake_capability_node src/${capability}_fake_capability_main.cpp)
  target_link_libraries(${capability}_fake_capability_node
    ${PROJECT_NAME}
    rclcpp::rclcpp
    lifecycle_msgs::lifecycle_msgs
  )
endforeach()

foreach(scenario alone collaboration)
  add_executable(${scenario}_mission_node src/main_${scenario}.cpp)
  target_link_libraries(${scenario}_mission_node
    rclcpp::rclcpp
    easyfleet_core::easyfleet_core
    easyfleet_mission_manager::easyfleet_mission_manager
  )
endforeach()

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
    navigation_fake_capability_node
    manipulation_fake_capability_node
    perception_fake_capability_node
    alone_mission_node
    collaboration_mission_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
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
  geometry_msgs
  sensor_msgs
  vision_msgs
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
