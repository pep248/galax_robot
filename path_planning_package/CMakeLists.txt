cmake_minimum_required(VERSION 3.8)
project(path_planning_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
ament_auto_find_build_dependencies()

# Add the service server executable
ament_auto_add_executable(path_planning_server
  src/path_planner_node.cpp
  src/path_planner_class.cpp
  src/goal_receiver_class.cpp) # dependencies) # dependencies
# Install the executable
install(TARGETS path_planning_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launchfile
install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
