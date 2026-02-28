from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("dofbot_moveit_config")
    bringup_share = FindPackageShare("dofbot_bringup")
    driver_pkg = FindPackageShare("dofbot_driver")

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the arm driver",
    )
    port = LaunchConfiguration("port")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "rsp.launch.py"])
        )
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "ros2_controllers.yaml"]),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
        output="screen",
    )

    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller"],
        output="screen",
    )

    return LaunchDescription([
        port_arg,
        rsp,
        Node(
            package="dofbot_driver",
            executable="dofbot_driver",
            name="dofbot_driver",
            output="screen",
            parameters=[{"port": port}],
        ),
        ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
    ])
