cmake_minimum_required(VERSION 3.8)
project(mentorpi_description)

find_package(ament_cmake REQUIRED)

# 纯描述包:安装 urdf / meshes / config
install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
