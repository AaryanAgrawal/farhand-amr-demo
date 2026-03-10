#!/usr/bin/env python3
"""Lightweight navigation state simulator — publishes nav status.

State machine: idle → navigating → goal_reached → idle
               idle → navigating → blocked → recovery → navigating
Faults: fault.drive_fault → emergency_stop
        LiDAR off → stuck in idle with warning
"""

import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

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

        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('fault.drive_fault', False)

        rate = self.get_parameter('rate_hz').value
        self.drive_fault = self.get_parameter('fault.drive_fault').value

        self.state = 'emergency_stop' if self.drive_fault else 'idle'
        self.tick = 0

        self.pub = self.create_publisher(String, 'nav_status', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(f'Navigation simulator started ({rate}Hz state machine)')
        log('State=idle. Waiting for goal.')
        if self.drive_fault:
            log('EMERGENCY — Drive fault detected. Navigation halted.')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.drive_fault':
                self.drive_fault = p.value
                if self.drive_fault:
                    self.state = 'emergency_stop'
                    log('EMERGENCY — Drive fault injected. Navigation halted.')
                else:
                    self.state = 'idle'
                    log('Drive fault cleared — navigation idle')
        return SetParametersResult(successful=True)

    def _check_lidar(self):
        try:
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
