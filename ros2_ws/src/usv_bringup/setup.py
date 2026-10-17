from glob import glob

from setuptools import setup

package_name = 'usv_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sean Johnson',
    maintainer_email='seanjohnson0807@gmail.com',
    description='GPS + BNO055 IMU bringup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_euler = usv_bringup.imu_euler:main',
        ],
    },
)
