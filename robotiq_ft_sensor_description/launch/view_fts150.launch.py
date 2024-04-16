from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# from ament_index_python.packages import get_package_share_directory
# import os, yaml

# ============================= NOT WORKING!!! =====================================================================


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_file]
            ),
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "name:=",
            "robotiq_fts150",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robotiq_ft_sensor_description"), "launch", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )

    nodes = [robot_state_publisher_node, rviz_node]

    return nodes


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package", default_value="robotiq_ft_sensor_description"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/example_use_robotiq_fts150.urdf.xacro",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("tf_prefix", default_value='""'))
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
