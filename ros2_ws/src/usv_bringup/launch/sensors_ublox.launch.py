import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('usv_bringup'), 'config')

    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            namespace='gps',
            output='screen',
            parameters=[os.path.join(config, 'ublox.yaml')],
            # Same topic names as nmea_navsat_driver in sensors.launch.py.
            # /gps/vel becomes TwistWithCovarianceStamped (nmea driver: TwistStamped).
            remappings=[
                ('~/fix', '/gps/fix'),
                ('~/fix_velocity', '/gps/vel'),
            ],
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
