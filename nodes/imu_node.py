#!/usr/bin/env python3
"""Simulated IMU — publishes Imu at 5Hz.

Fault mode: IMU_DEGRADED=1 → rate drops to 1Hz, noise 50x, yaw bias drift.
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import random
from datetime import datetime

LOG_PATH = os.path.expanduser('~/logs/sensor_health.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] IMU: {msg}\n')
    except Exception:
        pass


class ImuSimNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.pub = self.create_publisher(Imu, '/bcr_bot/imu', 10)
        self.degraded = os.environ.get('IMU_DEGRADED') == '1'
        rate = 1.0 if self.degraded else 5.0
        self.timer = self.create_timer(1.0 / rate, self.publish)
        self.yaw = 0.0
        self.yaw_bias = 0.0
        self.noise_sigma = 0.5 if self.degraded else 0.01

        mode = f'DEGRADED ({rate}Hz, noise σ={self.noise_sigma})' if self.degraded else f'normal ({rate}Hz)'
        self.get_logger().info(f'IMU simulator started ({mode})')
        if self.degraded:
            log(f'WARNING — Publishing rate degraded: {rate}Hz (expected 5.0Hz)')
            log(f'WARNING — Noise level elevated: σ={self.noise_sigma} (threshold: 0.1)')

    def publish(self):
        if self.degraded:
            self.yaw_bias += 0.0005
        self.yaw += random.gauss(0, 0.0001) + self.yaw_bias

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_frame'
        msg.orientation.z = math.sin(self.yaw / 2.0)
        msg.orientation.w = math.cos(self.yaw / 2.0)
        msg.orientation_covariance[0] = 0.001
        msg.orientation_covariance[4] = 0.001
        msg.orientation_covariance[8] = 0.001
        msg.angular_velocity.x = random.gauss(0, self.noise_sigma)
        msg.angular_velocity.y = random.gauss(0, self.noise_sigma)
        msg.angular_velocity.z = random.gauss(0, self.noise_sigma)
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
