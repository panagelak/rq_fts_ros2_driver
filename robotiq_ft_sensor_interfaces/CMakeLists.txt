cmake_minimum_required(VERSION 3.5)
project(robotiq_ft_sensor_interfaces)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} "msg/FTSensor.msg" "srv/SensorAccessor.srv" DEPENDENCIES
  builtin_interfaces std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
