from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robotiq_ft_sensor = Node(
        package="robotiq_ft_sensor_hardware",
        executable="robotiq_ft_sensor_standalone_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            {"max_retries": LaunchConfiguration("max_retries")},
            {"read_rate": LaunchConfiguration("read_rate")},
            {"ftdi_id": LaunchConfiguration("ftdi_id")},
            {"frame_id": LaunchConfiguration("frame_id")},
        ],
    )

    nodes = [robotiq_ft_sensor]

    return nodes


def generate_launch_description():
    # Declare Launch Arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    declared_arguments.append(DeclareLaunchArgument("max_retries", default_value="100"))
    declared_arguments.append(DeclareLaunchArgument("read_rate", default_value="10"))
    declared_arguments.append(DeclareLaunchArgument("ftdi_id", default_value=""))
    declared_arguments.append(
        DeclareLaunchArgument("frame_id", default_value="robotiq_ft_frame_id")
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
