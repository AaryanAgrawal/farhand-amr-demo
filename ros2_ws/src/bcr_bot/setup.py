from setuptools import setup
from glob import glob
import os

package_name = 'bcr_bot'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aaryan Agrawal',
    maintainer_email='aaryan@farhand.live',
    description='BCR-001 AMR simulator with runtime fault injection',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'battery_node = bcr_bot.battery_node:main',
            'depth_camera_node = bcr_bot.depth_camera_node:main',
            'stereo_camera_node = bcr_bot.stereo_camera_node:main',
            'drive_node = bcr_bot.drive_node:main',
            'imu_node = bcr_bot.imu_node:main',
            'lidar_2d_node = bcr_bot.lidar_2d_node:main',
            'nav_sim_node = bcr_bot.nav_sim_node:main',
            'payload_node = bcr_bot.payload_node:main',
        ],
    },
)
