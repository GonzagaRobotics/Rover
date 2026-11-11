import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive',
            executable='drive',
        ),
        Node(
            package='vision',
            executable='vision',
            parameters=[{
                "camera_index": 0,
            }]
        ),
        Node(
            package='video_source',
            executable='source',
            parameters=[{
                "remote_ip": "127.0.0.1",
            }]
        )
    ])
