#!/usr/bin/python3
"""Standalone LiDAR launch — start after LiDAR replacement.

Usage:
  ros2 launch bcr_bot lidar.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bcr_bot')

    sensor_params = LaunchConfiguration('sensor_params',
        default=os.path.join(pkg_share, 'config', 'sensor_params.yaml'))
    fault_params = LaunchConfiguration('fault_params',
        default=os.path.join(pkg_share, 'config', 'fault_params.yaml'))

    lidar = Node(
        package='bcr_bot',
        executable='lidar_2d_node',
        namespace='bcr_bot',
        parameters=[sensor_params, fault_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('sensor_params', default_value=sensor_params),
        DeclareLaunchArgument('fault_params', default_value=fault_params),
        lidar,
    ])
