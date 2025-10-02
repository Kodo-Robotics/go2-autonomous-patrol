cmake_minimum_required(VERSION 3.5)
project(go2_champ_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()