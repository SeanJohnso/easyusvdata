import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('usv_bringup'), 'config')
    ekf = os.path.join(config, 'ekf.yaml')

    return LaunchDescription([
        # Sensor mounting on the hull, zero offsets for now.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_bno055',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'bno055'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['--frame-id', 'base_link', '--child-frame-id', 'gps'],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[ekf],
            remappings=[('odometry/filtered', '/odometry/local')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[ekf],
            remappings=[('odometry/filtered', '/odometry/global')],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(config, 'navsat.yaml')],
            remappings=[
                ('imu', '/bno055/imu'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', '/odometry/global'),
                ('odometry/gps', '/odometry/gps'),
                ('gps/filtered', '/gps/filtered'),
            ],
        ),
    ])
