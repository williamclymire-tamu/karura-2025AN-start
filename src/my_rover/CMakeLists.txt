cmake_minimum_required(VERSION 3.5)
project(my_rover)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install URDF, config, and launch files
install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
