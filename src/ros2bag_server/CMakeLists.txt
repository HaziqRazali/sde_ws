cmake_minimum_required(VERSION 3.5)
project(ros2bag_server)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CollectRosbag.srv"
  "srv/SetTaskLabel.srv"
  "srv/SetEpisode.srv"
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python scripts # No, let setup.py install the Python package (entry points)
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  ros2bag_server/ros2bag_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
