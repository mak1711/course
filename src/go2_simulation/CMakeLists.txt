cmake_minimum_required(VERSION 3.8)
project(go2_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds meshes dae config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
