import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    'launch/rosbridge_websocket_launch.xml'
                )
            )
        ),
        Node(
            package='pathfinder',
            executable='pathfinder',
            parameters=[
                {'static_dir': '/home/robotics/static'},
                {'site_name': 'urc'}
            ]
        ),
        Node(
            package='auto_nav',
            executable='auto_nav',
        ),
        Node(
            package='core',
            executable='core',
        ),
        Node(
            package='nav_sensors_antenna',
            executable='nav_sensors_antenna',
        ),
        Node(
            package='drive',
            executable='drive',
        ),
        Node(
            package='aruco',
            executable='aruco',
            parameters=[
                {"camera_index": 0},
                {"camera_name": "c270"}
            ]
        ),
    ])
