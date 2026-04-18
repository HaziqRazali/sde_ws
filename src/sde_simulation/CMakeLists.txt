cmake_minimum_required(VERSION 3.5)
project(sde_simulation)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

ament_package()
