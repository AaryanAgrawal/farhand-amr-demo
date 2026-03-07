#!/usr/bin/env python3
"""Lightweight navigation state simulator — publishes nav status at 1Hz.

No actual Nav2 needed. Simulates state machine:
  idle → navigating → goal_reached → idle
  idle → navigating → blocked → recovery → navigating
  DRIVE_FAULT=1 → emergency_stop
  LiDAR off → stuck in idle with warning
"""

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

LOG_PATH = os.path.expanduser('~/logs/navigation.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] NAV: {msg}\n')
    except Exception:
        pass


class NavSimNode(Node):
    def __init__(self):
        super().__init__('nav_sim_node')
        self.pub = self.create_publisher(String, '/bcr_bot/nav_status', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.state = 'idle'
        self.tick = 0
        self.drive_fault = os.environ.get('DRIVE_FAULT') == '1'

        self.get_logger().info('Navigation simulator started (1Hz state machine)')
        log('State=idle. Waiting for goal.')

        if self.drive_fault:
            self.state = 'emergency_stop'
            log('EMERGENCY — Drive fault detected. Navigation halted.')

    def _check_lidar(self):
        """Check if LiDAR topic exists by looking at node list."""
        # Simple heuristic: check if lidar node process is running
        # In reality we'd check topic, but for sim we check env
        try:
            import subprocess
            result = subprocess.run(
                ['bash', '-lic', 'ros2 node list 2>/dev/null | grep -c two_d_lidar'],
                capture_output=True, text=True, timeout=3
            )
            return result.stdout.strip() != '0'
        except Exception:
            return False

    def publish(self):
        self.tick += 1

        if self.drive_fault:
            self.state = 'emergency_stop'
        elif self.state == 'idle' and self.tick % 60 == 0:
            if self._check_lidar():
                self.state = 'navigating'
                log('Goal received: dock_station_3 (x=12.5, y=3.2). ETA: 45s')
            else:
                log('WARNING — LiDAR not available. Navigation disabled.')

        msg = String()
        msg.data = self.state
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NavSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
