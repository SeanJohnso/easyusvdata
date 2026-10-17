import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('usv_bringup'), 'config')

    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_navsat_driver',
            namespace='gps',
            output='screen',
            parameters=[os.path.join(config, 'gps.yaml')],
        ),
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            output='screen',
            parameters=[os.path.join(config, 'bno055.yaml')],
        ),
        Node(
            package='usv_bringup',
            executable='imu_euler',
            name='imu_euler',
            output='screen',
        ),
    ])
