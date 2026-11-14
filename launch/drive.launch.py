import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive',
            executable='drive',
        ),
        ComposableNodeContainer(
            name='video_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_source',
                    plugin='CameraSource',
                ),
                ComposableNode(
                    package='video_source',
                    plugin='SourceNode',
                    parameters=[{
                        "remote_ip": "127.0.0.1",
                    }]
                ),
            ]
        )
    ])
