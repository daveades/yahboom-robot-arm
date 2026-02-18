from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480]
        }]
    )

    info_node = Node(
        package='dofbot_vision',
        executable='fake_camera_info',
        name='fake_camera_info',
        parameters=[{
            'publish_topic': '/camera/camera_info',
            'frame_id': 'camera_frame',
            'width': 640,
            'height': 480,
            'fx': 600.0,
            'fy': 600.0,
            'cx': 320.0,
            'cy': 240.0,
            'rate_hz': 10.0
        }]
    )

    aruco_node = Node(
        package='dofbot_vision',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'marker_size': 0.03,
            'marker_id': -1,
            'dictionary': 'DICT_4X4_50',
            'publish_tf': True,
            'tf_prefix': 'aruco_'
        }]
    )

    return LaunchDescription([
        camera_node,
        info_node,
        aruco_node
    ])
