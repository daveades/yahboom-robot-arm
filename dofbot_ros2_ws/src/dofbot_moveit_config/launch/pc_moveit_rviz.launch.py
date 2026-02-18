from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz",
        )
    )

    use_rviz = LaunchConfiguration("use_rviz")
    pkg_share = FindPackageShare("dofbot_bringup")

    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, "launch", "pc_moveit_rviz.launch.py"])
        ),
        launch_arguments={"use_rviz": use_rviz}.items(),
    )

    return LaunchDescription(declared_arguments + [include_launch])
