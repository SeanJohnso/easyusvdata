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
    description='GPS and IMU module test rig bringup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'check = usv_bringup.check:main',
            'ism330dhcx = usv_bringup.ism330dhcx:main',
            'lis3mdl = usv_bringup.lis3mdl:main',
            'mark = usv_bringup.mark:main',
            'mmc5983ma = usv_bringup.mmc5983ma:main',
            'mpu9250 = usv_bringup.mpu9250:main',
        ],
    },
)
