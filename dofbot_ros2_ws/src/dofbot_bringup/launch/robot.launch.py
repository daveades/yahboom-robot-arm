from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            default_value="false",
            description="Use joint_state_publisher (no hardware)",
        )
    )

    port = LaunchConfiguration("port")
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")

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

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=IfCondition(use_joint_state_publisher),
    )

    driver_node = Node(
        package="dofbot_driver",
        executable="dofbot_driver",
        name="dofbot_driver",
        output="screen",
        parameters=[{"port": port}],
        condition=UnlessCondition(use_joint_state_publisher),
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
        condition=IfCondition(use_rviz),
    )

    nodes = [
        robot_state_publisher_node,
        joint_state_publisher_node,
        driver_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
