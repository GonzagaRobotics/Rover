import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource


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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch/rs_launch.py'
                )
            ),
            launch_arguments={
                'camera_namespace': '/',
                'camera_name': 'rs',
                'pointcloud.enable': 'true',
                'depth_module.depth_profile': '1280x720x30',
                'decimation_filter.enable': 'true',
                'spatial_filter.enable': 'true',
                'temporal_filter.enable': 'true',
                'disparity_to_depth.enable': 'true',
                'disparity_filter.enable': 'true',
            }
        ),
        Node(
            package='pathfinder',
            executable='pathfinder',
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
