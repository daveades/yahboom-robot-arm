from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Real-hardware bringup: robot_state_publisher + serial driver.

    The driver itself provides the FollowJointTrajectory action servers
    MoveIt expects (arm_controller + gripper_controller) and executes
    trajectories as timed waypoints, so no ros2_control stack is needed
    on hardware. (Simulation still uses ros2_control — see demo.launch.py.)
    """
    bringup_share = FindPackageShare("dofbot_bringup")

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/ttyUSB0",
        description="Serial port for the arm driver",
    )
    port = LaunchConfiguration("port")

    startup_time_arg = DeclareLaunchArgument(
        "startup_time_ms",
        default_value="4000",
        description="Duration of the slow startup sync sweep in the driver",
    )
    startup_time_ms = ParameterValue(
        LaunchConfiguration("startup_time_ms"), value_type=int
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "rsp.launch.py"])
        )
    )

    return LaunchDescription([
        port_arg,
        startup_time_arg,
        rsp,
        Node(
            package="dofbot_driver",
            executable="dofbot_driver",
            name="dofbot_driver",
            output="screen",
            parameters=[{"port": port, "startup_time_ms": startup_time_ms}],
        ),
    ])
