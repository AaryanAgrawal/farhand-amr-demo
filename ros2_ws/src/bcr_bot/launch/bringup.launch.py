#!/usr/bin/python3
"""Main bringup launch — starts robot_state_publisher + 7 sim nodes (no LiDAR).

Usage:
  ros2 launch bcr_bot bringup.launch.py
  ros2 launch bcr_bot bringup.launch.py fault_params:=/path/to/custom_faults.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('bcr_bot')

    sensor_params = LaunchConfiguration('sensor_params',
        default=os.path.join(pkg_share, 'config', 'sensor_params.yaml'))
    fault_params = LaunchConfiguration('fault_params',
        default=os.path.join(pkg_share, 'config', 'fault_params.yaml'))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='bcr_bot',
        parameters=[{
            'robot_description': Command([
                'xacro ', os.path.join(pkg_share, 'urdf', 'bcr_bot.urdf.xacro'),
                ' two_d_lidar_enabled:=true',
                ' camera_enabled:=true',
                ' stereo_camera_enabled:=true',
            ])
        }],
    )

    common_args = dict(namespace='bcr_bot')

    battery = Node(package='bcr_bot', executable='battery_node', **common_args,
                   parameters=[sensor_params, fault_params])
    depth_camera = Node(package='bcr_bot', executable='depth_camera_node', **common_args,
                        parameters=[sensor_params, fault_params])
    stereo_camera = Node(package='bcr_bot', executable='stereo_camera_node', **common_args,
                         parameters=[sensor_params, fault_params])
    drive = Node(package='bcr_bot', executable='drive_node', **common_args,
                 parameters=[sensor_params, fault_params])
    imu = Node(package='bcr_bot', executable='imu_node', **common_args,
               parameters=[sensor_params, fault_params])
    nav_sim = Node(package='bcr_bot', executable='nav_sim_node', **common_args,
                   parameters=[sensor_params, fault_params])
    payload = Node(package='bcr_bot', executable='payload_node', **common_args,
                   parameters=[sensor_params, fault_params])

    return LaunchDescription([
        DeclareLaunchArgument('sensor_params', default_value=sensor_params),
        DeclareLaunchArgument('fault_params', default_value=fault_params),
        robot_state_publisher,
        battery,
        depth_camera,
        stereo_camera,
        drive,
        imu,
        nav_sim,
        payload,
    ])
