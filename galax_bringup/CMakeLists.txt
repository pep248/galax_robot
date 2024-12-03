cmake_minimum_required(VERSION 3.8)
project(galax_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# DEPENDENCIES

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amr-ros-config REQUIRED)
find_package(urg_node REQUIRED)
find_package(diffdrive_arduino REQUIRED)
# find_package(ros_arduino_bridge REQUIRED)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# # CPP
# # Add the action client executable
# ament_auto_add_executable(publisher_cpp src/publisher.cpp)
# # Install the executable
# install(TARGETS publisher_cpp
#   DESTINATION lib/${PROJECT_NAME}
# )
# # Add the action client executable
# ament_auto_add_executable(subscriber_cpp src/subscriber.cpp)
# # Install the executable
# install(TARGETS subscriber_cpp
#   DESTINATION lib/${PROJECT_NAME}
# )


# # PYTHON
# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})
# # Install Python executables
# install(PROGRAMS
#   scripts/publisher.py
#   scripts/subscriber.py
#   DESTINATION lib/${PROJECT_NAME}
# )


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    config
    rviz
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)