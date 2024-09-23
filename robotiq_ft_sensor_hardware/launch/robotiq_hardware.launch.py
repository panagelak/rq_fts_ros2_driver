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


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    namespace = LaunchConfiguration("namespace")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    tf_prefix = LaunchConfiguration("tf_prefix")

    max_retries = LaunchConfiguration("max_retries")
    read_rate = LaunchConfiguration("read_rate")
    ftdi_id = LaunchConfiguration("ftdi_id")
    use_fake_mode = LaunchConfiguration("use_fake_mode")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_file]
            ),
            " ",
            "name:=",
            "robotiq_fts300",
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_mode:=",
            use_fake_mode,
            " ",
            "max_retries:=",
            max_retries,
            " ",
            "read_rate:=",
            read_rate,
            " ",
            "ftdi_id:=",  # cannot pass empty string to hardware and will remove the first char if size is 1
            ftdi_id,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare(description_package.perform(context)),
            "launch",
            "view_robot.rviz",
        ]
    )

    # joint_state_publisher_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui",
    # )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
    )

    # no namespace
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # ROS2 Control

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package.perform(context)),
            controllers_file.perform(context),
        ]
    )

    control_node = Node(
        package="controller_manager",  # fake_hardware_interface
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, robot_controllers],
        remappings=[],
        output="both",
    )

    robotiq_force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "robotiq_force_torque_sensor_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    nodes = [
        robot_state_publisher_node,
        rviz_node,
        control_node,
        robotiq_force_torque_sensor_broadcaster_spawner,
    ]

    return nodes


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("namespace", default_value=""))
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package", default_value="robotiq_ft_sensor_hardware"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file", default_value="config/robotiq_controllers.yaml"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package", default_value="robotiq_ft_sensor_description"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/example_use_robotiq_ft300.urdf.xacro",
        )
    )
    declared_arguments.append(DeclareLaunchArgument("tf_prefix", default_value='""'))

    declared_arguments.append(DeclareLaunchArgument("max_retries", default_value="100"))
    declared_arguments.append(DeclareLaunchArgument("read_rate", default_value="10"))
    # cannot pass empty string to hardware and will remove the first char if size is 1
    declared_arguments.append(DeclareLaunchArgument("ftdi_id", default_value="_"))

    declared_arguments.append(
        DeclareLaunchArgument("use_fake_mode", default_value="false")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
