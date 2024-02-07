cmake_minimum_required(VERSION 3.5)
project(robotiq_ft_sensor_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch urdf meshes DESTINATION share/${PROJECT_NAME})
