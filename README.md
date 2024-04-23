# Robotiq Force Torque Sensor

## Overview

This repository implements

1) hardware sensor interface plugin for the robotiq force torque sensors

2) description for the robotiq ft300 and fts150

3) ros interfaces for the robotiq fts

It is tested for the Humble distribution

# Disclaimer 

Most of the code is directly copied by the original ros1 driver [Robotiq ros driver](https://github.com/ros-industrial/robotiq) so the original licenses and maintainers remain.

## How to build

```bash
# Step 1 Clone the repository

# Install debian dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --parallel-workers $(nproc) --base-paths src --symlink-install --event-handlers desktop_notification- status-   --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON

# Source
. install/setup.bash
```

## How to use

### Viewing the robot

```bash
# robotiq ft 300
ros2 launch robotiq_ft_sensor_description view_ft300.launch.py 
# ( not working )
ros2 launch robotiq_ft_sensor_description view_fts150.launch.py 
```

### Standalone robotiq node

```bash
ros2 launch robotiq_ft_sensor_hardware ft_sensor_standalone.launch.py
```

### Robotiq robot using ros2 control

```bash
# For real hardware
ros2 launch robotiq_ft_sensor_hardware robotiq_hardware.launch.py namespace:=robotiq
```

### Fake Mode
If you do not have the actual hardware but want to test your system a **fake mode** parameter is used that will bypass the actual connection to the hardware

```bash
# Fake pass through
ros2 launch robotiq_ft_sensor_hardware robotiq_hardware.launch.py use_fake_mode:=true namespace:=robotiq
```

## Controller configuration

The controller will publish a topic of type WrenchStamped on the topic name *controller_name/wrench* with the frame_id in the header as described here

```yaml
/**:
  controller_manager:
    ros__parameters:
      update_rate: 125  # Hz
      
      robotiq_force_torque_sensor_broadcaster:
        type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

  # will publish the robotiq_force_torque_sensor_broadcaster/wrench topic of type WrenchStamped with the given frame_id
  robotiq_force_torque_sensor_broadcaster:
    ros__parameters:
      sensor_name: robotiq_ft_sensor
      state_interface_names:
        - force.x
        - force.y
        - force.z
        - torque.x
        - torque.y
        - torque.z
      frame_id: robotiq_ft_frame_id
```

## ftdi_id
**ftdi_id** if empty string will automatically look for connected devices

## Admittance chained controller example
**In the robotiq_controllers.yaml** an example configuration on how to chain the admittance controller with the scale joint trajectory controller is placed

# Use it together with other hardware interfaces

The only requirement for the hardware interfaces to be dynamically loaded and run together by the controller manager is **to add to your urdf the ros2 control macro**

```xml
  <xacro:macro name="robotiq_fts_ros2_control" params="
    name
    use_fake_mode:=false
    max_retries:=1
    read_rate:=10
    ftdi_id:=''
  ">
    <ros2_control name="${name}robotiq_ft_sensor" type="sensor">
      <hardware>
          <plugin>robotiq_ft_sensor_hardware/RobotiqFTSensorHardware</plugin>
          <param name="use_fake_mode">${use_fake_mode}</param>
          <param name="max_retries">${max_retries}</param>
          <param name="read_rate">${read_rate}</param>
          <param name="ftdi_id">${ftdi_id}</param>
      </hardware>
      <sensor name="robotiq_ft_sensor">
        <state_interface name="force.x"/>
        <state_interface name="force.y"/>
        <state_interface name="force.z"/>
        <state_interface name="torque.x"/>
        <state_interface name="torque.y"/>
        <state_interface name="torque.z"/>
      </sensor>

    </ros2_control>

  </xacro:macro>
```

Example to use the robotiq fts hardware interface in combination with your robot

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="$(arg name)">

  <!-- args -->
  <xacro:arg name="name" default="robotiq_ft_sensor"/> <!-- your robot name -->
  <xacro:arg name="use_fake_mode" default="false"/>
  <xacro:arg name="max_retries" default="100"/>
  <xacro:arg name="read_rate" default="10"/>
  <xacro:arg name="ftdi_id" default=""/>
  <xacro:arg name="tf_prefix" default=""/>

  <!-- import robotiq ros2 control macro -->
  <xacro:include filename="$(find robotiq_ft_sensor_description)/urdf/robotiq_fts.ros2_control.xacro" />

  <!-- robotiq sensor hardware -->
  <xacro:robotiq_fts_ros2_control
      name="$(arg name)"
      use_fake_mode="$(arg use_fake_mode)"
      max_retries="$(arg max_retries)"
      read_rate="$(arg read_rate)"
      ftdi_id="$(arg ftdi_id)"
  />

</robot>
```

## Additional Info

The hardware interface creates an anonymous self spinning node to implement the **service SensorAccessor** in the service name **robotiq_ft_sensor_acc** instead of creating an additional controller.

Furthermore **an optional WrenchStamped subscriber is created which will append the values** received to the actual data from the sensor for test purposes.

# Disclaimer 

Most of the code is directly copied by the original ros1 driver [Robotiq ros driver](https://github.com/ros-industrial/robotiq) so the original licenses and maintainers remain.
