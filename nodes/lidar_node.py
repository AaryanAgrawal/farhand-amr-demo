#!/usr/bin/env python3
"""Simulated Livox Mid-360 LiDAR — publishes PointCloud2 at 10Hz + IMU at 200Hz.

The real Mid-360 has a built-in ICM40609 IMU that publishes on /livox/imu.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu
import numpy as np
import struct


class LidarSimNode(Node):
    def __init__(self):
        super().__init__('livox_mid360_node')
        self.pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 50)
        self.timer = self.create_timer(0.1, self.publish)  # 10 Hz
        self.imu_timer = self.create_timer(0.005, self.publish_imu)  # 200 Hz
        self.get_logger().info('Livox Mid-360 LiDAR simulator started (10Hz, 200k pts + 200Hz IMU)')

    def publish(self):
        n = 200000
        angles = np.random.uniform(0, 2 * np.pi, n).astype(np.float32)
        ranges = np.random.uniform(0.5, 30.0, n).astype(np.float32)
        elev = np.random.uniform(-0.12, 0.91, n).astype(np.float32)  # -7° to 52°

        x = ranges * np.cos(elev) * np.cos(angles)
        y = ranges * np.cos(elev) * np.sin(angles)
        z = ranges * np.sin(elev)
        intensity = np.random.uniform(0, 255, n).astype(np.float32)

        points = np.zeros(n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
        points['x'], points['y'], points['z'], points['i'] = x, y, z, intensity

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = points.tobytes()
        msg.is_dense = True
        self.pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'livox_frame'

        # ICM40609 simulated noise: gyro ±0.01 rad/s, accel ±0.1 m/s²
        msg.angular_velocity.x = float(np.random.normal(0, 0.01))
        msg.angular_velocity.y = float(np.random.normal(0, 0.01))
        msg.angular_velocity.z = float(np.random.normal(0, 0.01))

        msg.linear_acceleration.x = float(np.random.normal(0, 0.1))
        msg.linear_acceleration.y = float(np.random.normal(0, 0.1))
        msg.linear_acceleration.z = float(np.random.normal(9.81, 0.1))

        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = LidarSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
