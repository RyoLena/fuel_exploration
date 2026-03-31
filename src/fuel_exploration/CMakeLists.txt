cmake_minimum_required(VERSION 3.8)
project(fuel_exploration)

set(CMAKE_CXX_STANDARD 17)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(fuel_exploration_common
  src/common/navigation_utils.cpp
  src/common/occupancy_grid_utils.cpp
  src/common/robot_pose_transform.cpp
)
target_include_directories(fuel_exploration_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  fuel_exploration_common
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

add_library(fuel_exploration_frontier
  src/frontier/frontier_algorithms.cpp
  src/frontier/frontier_goal_selection.cpp
  src/frontier/frontier_map_processing.cpp
  src/frontier/frontier_path_messages.cpp
  src/frontier/frontier_visualization.cpp
)
target_include_directories(fuel_exploration_frontier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(fuel_exploration_frontier fuel_exploration_common)
ament_target_dependencies(
  fuel_exploration_frontier
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
)

add_library(fuel_exploration_controller_support
  src/controller/path_tracking.cpp
)
target_include_directories(fuel_exploration_controller_support PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  fuel_exploration_controller_support
  fuel_exploration_common
)
ament_target_dependencies(
  fuel_exploration_controller_support
  geometry_msgs
  nav_msgs
)

add_executable(frontier_detector src/frontier_detector.cpp)
target_link_libraries(
  frontier_detector
  fuel_exploration_common
  fuel_exploration_frontier
)
ament_target_dependencies(frontier_detector geometry_msgs rclcpp nav_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)

add_executable(exploration_controller src/exploration_controller.cpp)
target_link_libraries(
  exploration_controller
  fuel_exploration_common
  fuel_exploration_controller_support
)
ament_target_dependencies(exploration_controller geometry_msgs rclcpp nav_msgs tf2 tf2_geometry_msgs tf2_ros)

install(
  TARGETS
    fuel_exploration_common
    fuel_exploration_frontier
    fuel_exploration_controller_support
    frontier_detector
    exploration_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
endif()

ament_package()
