import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_sensor_bridge',
            executable='mobile_sensor_bridge_node',
            name='mobile_sensor_bridge',
            output='screen'
        ),
        Node(
            package='mobile_sensor_bridge',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
