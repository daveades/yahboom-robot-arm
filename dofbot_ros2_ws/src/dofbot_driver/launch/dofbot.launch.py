from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB0",
            description="Serial port for the arm",
        )
    )

    port = LaunchConfiguration("port")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dofbot_description"), "urdf", "dofbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    driver_node = Node(
        package="dofbot_driver",
        executable="dofbot_driver",
        name="dofbot_driver",
        output="screen",
        parameters=[{"port": port}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("dofbot_description"), "rviz", "robot.rviz"]
            ),
        ],
    )

    nodes = [
        robot_state_publisher_node,
        driver_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
