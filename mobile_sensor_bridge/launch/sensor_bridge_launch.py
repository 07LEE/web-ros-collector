import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument('port', default_value='8443', description='HTTPS Server Port')
    image_topic_arg = DeclareLaunchArgument('image_topic', default_value='/image_raw/compressed', description='Image Topic Name')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data_raw', description='IMU Topic Name')
    camera_info_url_arg = DeclareLaunchArgument('camera_info_url', default_value='', description='Camera Calibration YAML URL')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to start rviz2')
    bag_output_dir_arg = DeclareLaunchArgument('bag_output_dir', default_value='', description='Rosbag Output Directory')
    heartbeat_timeout_arg = DeclareLaunchArgument('heartbeat_timeout', default_value='10.0', description='Heartbeat timeout in seconds for auto recording stop')

    return LaunchDescription([
        port_arg,
        image_topic_arg,
        imu_topic_arg,
        camera_info_url_arg,
        use_rviz_arg,
        bag_output_dir_arg,
        heartbeat_timeout_arg,
        Node(
            package='mobile_sensor_bridge',
            executable='mobile_sensor_bridge_node',
            name='mobile_sensor_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'image_topic': LaunchConfiguration('image_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'bag_output_dir': LaunchConfiguration('bag_output_dir'),
                'heartbeat_timeout': LaunchConfiguration('heartbeat_timeout'),
            }]
        ),
        Node(
            package='mobile_sensor_bridge',
            executable='camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            parameters=[{
                'camera_info_url': LaunchConfiguration('camera_info_url'),
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz'))
        )
    ])

