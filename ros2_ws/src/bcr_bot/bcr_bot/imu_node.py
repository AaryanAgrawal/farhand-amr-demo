#!/usr/bin/env python3
"""Simulated IMU — publishes Imu messages.

Fault: fault.degraded → rate drops to 1Hz, noise 50x, yaw bias drift.
"""

import os
import math
import random
from datetime import datetime

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Imu

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

        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('noise_sigma', 0.01)
        self.declare_parameter('fault.degraded', False)

        self.nominal_rate = self.get_parameter('rate_hz').value
        self.nominal_sigma = self.get_parameter('noise_sigma').value
        self.degraded = self.get_parameter('fault.degraded').value

        self.yaw = 0.0
        self.yaw_bias = 0.0

        rate = 1.0 if self.degraded else self.nominal_rate
        self.noise_sigma = 0.5 if self.degraded else self.nominal_sigma

        self.pub = self.create_publisher(Imu, 'imu', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.add_on_set_parameters_callback(self._on_param_change)

        mode = f'DEGRADED ({rate}Hz, noise σ={self.noise_sigma})' if self.degraded else f'normal ({rate}Hz)'
        self.get_logger().info(f'IMU simulator started ({mode})')
        if self.degraded:
            log(f'WARNING — Publishing rate degraded: {rate}Hz (expected {self.nominal_rate}Hz)')
            log(f'WARNING — Noise level elevated: σ={self.noise_sigma} (threshold: 0.1)')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.degraded':
                self.degraded = p.value
                rate = 1.0 if self.degraded else self.nominal_rate
                self.noise_sigma = 0.5 if self.degraded else self.nominal_sigma
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / rate, self.publish)
                if self.degraded:
                    log(f'FAULT INJECTED — rate degraded to {rate}Hz')
                else:
                    self.yaw_bias = 0.0
                    log('Fault cleared — normal IMU rate restored')
        return SetParametersResult(successful=True)

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
