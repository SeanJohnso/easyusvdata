"""Start every GPS and IMU module in config/rig.yaml, plus a static TF per sensor frame.

Driver settings come from config/<file>.yaml; the port, baud, bus, address and
frame of each module come from rig.yaml and override those files.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml

CONFIG = os.path.join(get_package_share_directory('usv_bringup'), 'config')

# IMU driver name in rig.yaml -> (package, executable, address parameter name)
IMU_DRIVERS = {
    'bno055': ('bno055', 'bno055', 'i2c_addr'),
    'icm20948': ('icm20948_ros2', 'icm20948_raw', 'i2c_address'),
    'mpu9250': ('usv_bringup', 'mpu9250', 'i2c_addr'),
    'ism330dhcx': ('usv_bringup', 'ism330dhcx', 'i2c_addr'),
    'mmc5983ma': ('usv_bringup', 'mmc5983ma', 'i2c_addr'),
    'lis3mdl': ('usv_bringup', 'lis3mdl', 'i2c_addr'),
}


def static_tf(frame, pose):
    args = ['--frame-id', 'base_link', '--child-frame-id', frame]
    for k, v in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), pose):
        args += [f'--{k}', str(v)]
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name=f'tf_{frame}', arguments=args)


def imu_node(driver, iid, bus, addr, frame):
    package, executable, addr_param = IMU_DRIVERS[driver]
    return Node(
        package=package,
        executable=executable,
        name=driver,
        namespace=f'imu/{iid}',
        output='screen',
        parameters=[os.path.join(CONFIG, f'{driver}.yaml'),
                    {'i2c_bus': bus, addr_param: addr, 'frame_id': frame}],
    )


def generate_launch_description():
    with open(os.path.join(CONFIG, 'rig.yaml')) as f:
        rig = yaml.safe_load(f)
    actions = []

    for gid, g in rig['gps'].items():
        actions.append(Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            namespace=f'gps/{gid}',
            output='screen',
            parameters=[os.path.join(CONFIG, g['config']),
                        {'device': g['device'], 'uart1.baudrate': g['baud'],
                         'frame_id': f'gps_{gid}'}],
            remappings=[('~/fix', f'/gps/{gid}/fix'),
                        ('~/fix_velocity', f'/gps/{gid}/vel')],
        ))
        actions.append(static_tf(f'gps_{gid}', g['position']))

    for iid, m in rig['imu'].items():
        actions.append(imu_node(m['driver'], iid, m['i2c_bus'], m['i2c_addr'], f'imu_{iid}'))
        actions.append(static_tf(f'imu_{iid}', m['pose']))
        if 'mag_driver' in m:
            actions.append(imu_node(m['mag_driver'], iid, m['i2c_bus'], m['mag_i2c_addr'],
                                    f'imu_{iid}_mag'))
            actions.append(static_tf(f'imu_{iid}_mag', m['mag_pose']))

    return LaunchDescription(actions)
