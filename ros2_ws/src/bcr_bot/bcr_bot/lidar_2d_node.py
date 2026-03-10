#!/usr/bin/env python3
"""Simulated 2D LiDAR — publishes LaserScan.

Faults:
  fault.partial_occlusion → 45° dead zone (samples 90-135 = 0.0)
  fault.intermittent → stops publishing for 5s every ~30s
"""

import os
import time
import math
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan

LOG_PATH = os.path.expanduser('~/logs/sensor_health.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] LIDAR: {msg}\n')
    except Exception:
        pass


class TwoDLidarNode(Node):
    def __init__(self):
        super().__init__('two_d_lidar_node')

        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('num_samples', 361)
        self.declare_parameter('range_min', 0.55)
        self.declare_parameter('range_max', 16.0)
        self.declare_parameter('fault.partial_occlusion', False)
        self.declare_parameter('fault.intermittent', False)

        self.rate = self.get_parameter('rate_hz').value
        self.num_samples = self.get_parameter('num_samples').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.occlusion = self.get_parameter('fault.partial_occlusion').value
        self.intermittent = self.get_parameter('fault.intermittent').value

        self.start_time = time.time()
        self.dropout_logged = False

        self.pub = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.publish)

        self.add_on_set_parameters_callback(self._on_param_change)

        faults = []
        if self.occlusion:
            faults.append('partial occlusion (45° dead zone)')
        if self.intermittent:
            faults.append('intermittent dropout (5s every 30s)')
        mode = ', '.join(faults) if faults else 'normal'
        self.get_logger().info(f'2D LiDAR started ({self.rate}Hz, {self.num_samples} samples, {mode})')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.partial_occlusion':
                self.occlusion = p.value
                log(f'Partial occlusion {"ACTIVE" if p.value else "cleared"}')
            elif p.name == 'fault.intermittent':
                self.intermittent = p.value
                self.start_time = time.time()
                log(f'Intermittent dropout {"ACTIVE" if p.value else "cleared"}')
        return SetParametersResult(successful=True)

    def publish(self):
        if self.intermittent:
            elapsed = time.time() - self.start_time
            cycle = elapsed % 35.0
            if cycle >= 30.0:
                if not self.dropout_logged:
                    log('WARNING — Data dropout detected. Duration: ~5s')
                    self.dropout_logged = True
                return
            else:
                self.dropout_logged = False

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'two_d_lidar'

        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2.0 * math.pi) / (self.num_samples - 1)
        msg.time_increment = (1.0 / self.rate) / self.num_samples
        msg.scan_time = 1.0 / self.rate
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        ranges = np.random.uniform(1.0, 12.0, self.num_samples).astype(np.float32)
        intensities = np.random.uniform(0, 255, self.num_samples).astype(np.float32)

        if self.occlusion:
            ranges[90:136] = 0.0
            intensities[90:136] = 0.0

        msg.ranges = ranges.tolist()
        msg.intensities = intensities.tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TwoDLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
