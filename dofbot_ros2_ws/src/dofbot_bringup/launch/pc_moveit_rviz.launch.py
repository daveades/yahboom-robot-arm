from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    MoveItConfigsBuilder(
        "yahboom_dofbot", package_name="dofbot_moveit_config"
    ).to_moveit_configs()

    pkg_share = FindPackageShare("dofbot_moveit_config")
    bringup_share = FindPackageShare("dofbot_bringup")
    params_file_path = (
        get_package_share_directory("dofbot_moveit_config")
        + "/config/moveit_params.yaml"
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "static_virtual_joint_tfs.launch.py"])
            )
        )
    )
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([bringup_share, "launch", "rsp.launch.py"])
            )
        )
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            params_file_path,
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "capabilities": "",
                "disable_capabilities": "",
                "monitor_dynamics": False,
            },
        ],
    )
    ld.add_action(move_group_node)

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution([pkg_share, "config", "moveit.rviz"]),
        ],
        parameters=[params_file_path],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    ld.add_action(rviz_node)

    return ld
