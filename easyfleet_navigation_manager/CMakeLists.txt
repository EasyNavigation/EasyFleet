cmake_minimum_required(VERSION 3.20)
project(easyfleet_navigation_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(easynav_costmap_maps_manager REQUIRED)
find_package(easynav_routes_maps_manager REQUIRED)
find_package(easynav_interfaces REQUIRED)
find_package(easyfleet_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/costmap_map_publisher.cpp
  src/${PROJECT_NAME}/routes_publisher.cpp
  src/${PROJECT_NAME}/conflict_detector.cpp
  src/${PROJECT_NAME}/robot_navigation_watcher.cpp
  src/${PROJECT_NAME}/navigation_manager_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  interactive_markers::interactive_markers
  tf2_ros::tf2_ros
  ament_index_cpp::ament_index_cpp
  easynav_costmap_maps_manager::easynav_costmap_maps_manager
  easynav_routes_maps_manager::easynav_routes_maps_manager_lib
  ${easynav_interfaces_TARGETS}
  ${easyfleet_interfaces_TARGETS}
)

add_executable(navigation_manager_node src/main_navigation_manager.cpp)
target_link_libraries(navigation_manager_node
  rclcpp::rclcpp
  ${PROJECT_NAME}
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
  TARGETS navigation_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf2_ros
  ament_index_cpp
  easynav_costmap_maps_manager
  easynav_routes_maps_manager
  easynav_interfaces
  easyfleet_interfaces
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
