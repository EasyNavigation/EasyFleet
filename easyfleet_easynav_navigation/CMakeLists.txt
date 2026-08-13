cmake_minimum_required(VERSION 3.20)
project(easyfleet_easynav_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(easynav_system REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# The Navigate BT node: the one resource this package provides for use by
# EasyNav, built as a BT.CPP plugin (BT_REGISTER_NODES, loaded by path via
# BT::BehaviorTreeFactory::registerFromPlugin()) rather than a library a
# capability links against at compile time -- any deployment package can
# load it by path alongside its own bookend nodes without a compile-time
# dependency on this package's implementation.
add_library(easynav_navigate_bt_node SHARED
  src/${PROJECT_NAME}/navigate.cpp
)
target_include_directories(easynav_navigate_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(easynav_navigate_bt_node PUBLIC
  behaviortree_cpp::behaviortree_cpp
  easynav_system::easynav_system
  geometry_msgs::geometry_msgs
)
target_compile_definitions(easynav_navigate_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS easynav_navigate_bt_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
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
