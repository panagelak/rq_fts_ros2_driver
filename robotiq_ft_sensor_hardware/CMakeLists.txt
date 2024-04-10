cmake_minimum_required(VERSION 3.5)
project(robotiq_ft_sensor_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    geometry_msgs
    rclcpp hardware_interface pluginlib
    realtime_tools
    robotiq_ft_sensor_interfaces
    std_srvs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# COMPILE

# include
include_directories(include)

# === Private library
add_library(robotiq_ft_sensor src/rq_sensor_com.cpp src/rq_sensor_state.cpp)
ament_target_dependencies(robotiq_ft_sensor ${dependencies})

# === Shared Library
add_library(robotiq_ft_sensor_hardware SHARED
            src/robotiq_ft_sensor_hardware.cpp)
target_include_directories(robotiq_ft_sensor_hardware PRIVATE include)
ament_target_dependencies(robotiq_ft_sensor_hardware
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(robotiq_ft_sensor_hardware robotiq_ft_sensor)

# === Export hardware plugin
pluginlib_export_plugin_description_file(hardware_interface
                                         robotiq_ft_sensor_hardware.xml)

# === standalone executables
add_executable(robotiq_ft_sensor_standalone_node
               nodes/robotiq_ft_sensor_standalone_node.cpp)
ament_target_dependencies(robotiq_ft_sensor_standalone_node
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(robotiq_ft_sensor_standalone_node robotiq_ft_sensor)

add_executable(test_robotiq_ft_sensor_node
               nodes/test_robotiq_ft_sensor_node.cpp)
ament_target_dependencies(test_robotiq_ft_sensor_node
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(test_robotiq_ft_sensor_node robotiq_ft_sensor)

add_executable(test_add_wrench_pub nodes/test_add_wrench_pub.cpp)
ament_target_dependencies(test_add_wrench_pub ${THIS_PACKAGE_INCLUDE_DEPENDS})

# INSTALL
install(DIRECTORY include/ DESTINATION include)

install(FILES robotiq_ft_sensor_hardware.xml
        DESTINATION share/robotiq_ft_sensor_hardware)

install(
  TARGETS robotiq_ft_sensor_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS robotiq_ft_sensor_standalone_node test_robotiq_ft_sensor_node
                test_add_wrench_pub RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# EXPORTS
ament_export_include_directories(include)
ament_export_libraries(robotiq_ft_sensor_hardware)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
# package
ament_package()
