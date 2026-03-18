from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='vision',
            parameters=[
                {"camera_index": 0},
                {"camera_name": "c270"}
            ]
        ),
        Node(
            package="aruco",
            executable="aruco"
        ),
        Node(
            package="object_detect",
            executable="object_detect"
        )
    ])
