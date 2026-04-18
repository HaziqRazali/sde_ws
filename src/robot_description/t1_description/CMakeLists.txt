cmake_minimum_required(VERSION 3.5)
project(t1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install here to be found by Genesis in workspace_ws/install/
install(
  DIRECTORY meshes/
  DESTINATION meshes
)

# Install here to be found by Genesis in binary installation at /opt/ros/humble/
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/urdf/${PROJECT_NAME}/meshes
)

ament_package()
