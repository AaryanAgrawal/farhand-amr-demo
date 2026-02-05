#!/usr/bin/env python3
"""Simulated IMU — publishes Imu at 100Hz."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import random


class ImuSimNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.01, self.publish)  # 100 Hz
        self.yaw = 0.0
        self.get_logger().info('IMU simulator started (100Hz)')

    def publish(self):
        self.yaw += random.gauss(0, 0.0001)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.z = math.sin(self.yaw / 2.0)
        msg.orientation.w = math.cos(self.yaw / 2.0)
        msg.orientation_covariance[0] = 0.001
        msg.orientation_covariance[4] = 0.001
        msg.orientation_covariance[8] = 0.001
        msg.angular_velocity.x = random.gauss(0, 0.01)
        msg.angular_velocity.y = random.gauss(0, 0.01)
        msg.angular_velocity.z = random.gauss(0, 0.01)
        msg.linear_acceleration.x = random.gauss(0, 0.05)
        msg.linear_acceleration.y = random.gauss(0, 0.05)
        msg.linear_acceleration.z = 9.81 + random.gauss(0, 0.05)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
