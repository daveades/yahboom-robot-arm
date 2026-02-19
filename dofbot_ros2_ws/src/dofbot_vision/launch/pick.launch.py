from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = LaunchConfiguration("params")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("dofbot_vision"), "config", "picking.yaml"]
                ),
            ),
            Node(
                package="dofbot_vision",
                executable="pick_from_detections",
                name="pick_from_detections",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
