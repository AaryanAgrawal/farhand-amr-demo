#!/usr/bin/env python3
"""Simulated 2D LiDAR — publishes LaserScan at 30Hz.

Fault modes:
  LIDAR_PARTIAL_OCCLUSION=1 → 45° dead zone (samples 90-135 = 0.0)
  LIDAR_INTERMITTENT=1 → stops publishing for 5s every ~30s
"""

import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from datetime import datetime

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
        self.pub = self.create_publisher(LaserScan, '/bcr_bot/scan', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish)  # 30 Hz

        self.occlusion = os.environ.get('LIDAR_PARTIAL_OCCLUSION') == '1'
        self.intermittent = os.environ.get('LIDAR_INTERMITTENT') == '1'
        self.start_time = time.time()
        self.dropout_logged = False

        faults = []
        if self.occlusion:
            faults.append('partial occlusion (45° dead zone)')
        if self.intermittent:
            faults.append('intermittent dropout (5s every 30s)')
        mode = ', '.join(faults) if faults else 'normal'
        self.get_logger().info(f'2D LiDAR started (30Hz, 361 samples, {mode})')

    def publish(self):
        # Intermittent dropout: 5s silence every 30s
        if self.intermittent:
            elapsed = time.time() - self.start_time
            cycle = elapsed % 35.0
            if cycle >= 30.0:
                if not self.dropout_logged:
                    log(f'WARNING — Data dropout detected. Duration: ~5s')
                    self.dropout_logged = True
                return
            else:
                self.dropout_logged = False

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'

        num_samples = 361
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = (2.0 * math.pi) / (num_samples - 1)
        msg.time_increment = (1.0 / 30.0) / num_samples
        msg.scan_time = 1.0 / 30.0
        msg.range_min = 0.55
        msg.range_max = 16.0

        ranges = np.random.uniform(1.0, 12.0, num_samples).astype(np.float32)
        intensities = np.random.uniform(0, 255, num_samples).astype(np.float32)

        # Partial occlusion: 45° dead zone
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
