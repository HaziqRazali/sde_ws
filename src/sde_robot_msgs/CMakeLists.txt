cmake_minimum_required(VERSION 3.5)
project(sde_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetTaskLabel.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
