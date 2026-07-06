from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Start the Arm Driver (via its own launch file)
    arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dofbot_driver'),
                'launch',
                'dofbot.launch.py'
            ])
        ])
    )
    
    # 2. Start the Camera Driver
    # Note: Requires 'ros-humble-v4l2-camera' installed on Pi
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480]
        }]
    )
    
    # 3. Start our Color Sorter
    vision_node = Node(
        package='dofbot_vision',
        executable='color_sorter',
        name='color_sorter',
        output='screen'
    )

    return LaunchDescription([
        arm_launch,
        camera_node,
        vision_node
    ])
