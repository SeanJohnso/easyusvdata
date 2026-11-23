"""Start every GPS and IMU module in config/rig.yaml, plus a static TF per sensor frame.

Driver settings come from config/<file>.yaml; the port, baud, bus, address and
frame of each module come from rig.yaml and override those files.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import yaml

CONFIG = os.path.join(get_package_share_directory('usv_bringup'), 'config')

# Seconds between starting one module (driver + its static TFs) and the next.
# Starting every node at once dipped the Pi's 5 V input low enough to cut power
# (2026-11-22: ~4.9 V idle on this supply, undervoltage under a 2-core load).
STAGGER = 1.0

# GPS drivers restart this many seconds after dying. The ublox driver aborts when its
# one MON-VER poll at start-up gets no reply (2026-11-23: the SAM-M8Q missed the whole
# dynamic test that way); a retry usually works.
GPS_RESPAWN_DELAY = 2.0

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
    modules = []   # one list of nodes per module, started STAGGER s apart

    for gid, g in rig['gps'].items():
        if g.get('driver') == 'nmea':
            # nmea_navsat_driver publishes fix and vel relative to its namespace.
            modules.append([Node(
                package='nmea_navsat_driver',
                executable='nmea_serial_driver',
                name='nmea_navsat_driver',
                namespace=f'gps/{gid}',
                output='screen',
                respawn=True,
                respawn_delay=GPS_RESPAWN_DELAY,
                parameters=[os.path.join(CONFIG, g['config']),
                            {'port': g['device'], 'baud': g['baud'], 'frame_id': f'gps_{gid}'}],
            ), static_tf(f'gps_{gid}', g['position'])])
            continue
        modules.append([Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            namespace=f'gps/{gid}',
            output='screen',
            respawn=True,
            respawn_delay=GPS_RESPAWN_DELAY,
            parameters=[os.path.join(CONFIG, g['config']),
                        {'device': g['device'], 'uart1.baudrate': g['baud'],
                         'frame_id': f'gps_{gid}'}],
            remappings=[('~/fix', f'/gps/{gid}/fix'),
                        ('~/fix_velocity', f'/gps/{gid}/vel')],
        ), static_tf(f'gps_{gid}', g['position'])])

    for iid, m in rig['imu'].items():
        nodes = [imu_node(m['driver'], iid, m['i2c_bus'], m['i2c_addr'], f'imu_{iid}'),
                 static_tf(f'imu_{iid}', m['pose'])]
        if 'mag_driver' in m:
            nodes += [imu_node(m['mag_driver'], iid, m['i2c_bus'], m['mag_i2c_addr'],
                               f'imu_{iid}_mag'),
                      static_tf(f'imu_{iid}_mag', m['mag_pose'])]
        modules.append(nodes)

    return LaunchDescription([TimerAction(period=i * STAGGER, actions=nodes)
                              for i, nodes in enumerate(modules)])
