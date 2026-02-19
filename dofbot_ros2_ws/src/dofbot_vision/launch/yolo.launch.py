from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_topic",
            default_value="/image_raw",
            description="Input image topic",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "model",
            default_value="yolov8n.pt",
            description="YOLO model path or name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Device (cpu or cuda:0)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "allowed_classes",
            default_value="",
            description="Comma-separated class names or IDs to allow",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "blocked_classes",
            default_value="",
            description="Comma-separated class names or IDs to block",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "min_area_ratio",
            default_value="0.0",
            description="Minimum bbox area ratio (0-1)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "max_area_ratio",
            default_value="1.0",
            description="Maximum bbox area ratio (0-1)",
        )
    )

    node = Node(
        package="dofbot_vision",
        executable="yolo_detector",
        name="yolo_detector",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "model": LaunchConfiguration("model"),
                "device": LaunchConfiguration("device"),
                "allowed_classes": LaunchConfiguration("allowed_classes"),
                "blocked_classes": LaunchConfiguration("blocked_classes"),
                "min_area_ratio": LaunchConfiguration("min_area_ratio"),
                "max_area_ratio": LaunchConfiguration("max_area_ratio"),
            }
        ],
    )

    return LaunchDescription(declared_arguments + [node])
