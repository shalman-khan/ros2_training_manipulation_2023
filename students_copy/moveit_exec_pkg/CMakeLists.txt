cmake_minimum_required(VERSION 3.8)
project(moveit_exec_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

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

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)


add_executable(moveit_exec_interface src/moveit_exec_interface.cpp)
target_include_directories(moveit_exec_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_exec_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_exec_interface
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS moveit_exec_interface
  DESTINATION lib/${PROJECT_NAME})

#add_executable(moveit_spawn_collision_object src/moveit_spawn_collision_object.cpp)
#target_include_directories(moveit_spawn_collision_object PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(moveit_spawn_collision_object PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
#ament_target_dependencies(
#  moveit_spawn_collision_object
#  "moveit_ros_planning_interface"
#  "rclcpp"
#  "shape_msgs"
#  "geometric_shapes"
#  "moveit_visual_tools"
#  "shape_msgs"
#  "ros2_aruco_interfaces"
#)
#
#install(TARGETS moveit_spawn_collision_object
#  DESTINATION lib/${PROJECT_NAME})

ament_package()
