cmake_minimum_required(VERSION 3.5)
project(my_rover)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install launch files
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/)

# Install URDF files
install(DIRECTORY urdf
        DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
