from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    robotiq_test_sensor = Node(
        package="robotiq_ft_sensor",
        executable="rq_test_sensor",
    )

    nodes = [robotiq_test_sensor]

    return nodes


def generate_launch_description():
    # Declare Launch Arguments
    declared_arguments = []
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
