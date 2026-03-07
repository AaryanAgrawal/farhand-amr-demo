#!/usr/bin/env python3
"""Simulated battery monitor — publishes BatteryState at 1Hz.

Fault mode: BATTERY_CRITICAL=1 → voltage 10.2V, 3%, NOT_CHARGING, jitter when <5%.
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import random
from datetime import datetime

LOG_PATH = os.path.expanduser('~/logs/battery.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] BATTERY: {msg}\n')
    except Exception:
        pass


class BatterySimNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.pub = self.create_publisher(BatteryState, '/bcr_bot/battery_state', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.critical = os.environ.get('BATTERY_CRITICAL') == '1'
        self.pct = 0.03 if self.critical else 0.85
        self.last_threshold = None
        mode = 'CRITICAL (10.2V, 3%)' if self.critical else 'normal (85%)'
        self.get_logger().info(f'Battery monitor started (1Hz, {mode})')
        log(f'Node started. Mode: {mode}')

    def publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if self.critical:
            jitter = random.uniform(-0.5, 0.5) if self.pct < 0.05 else 0.0
            msg.voltage = 10.2 + jitter
            msg.current = 0.0
            msg.percentage = self.pct
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        else:
            msg.voltage = 11.1 + (self.pct * 1.5)
            msg.current = -1.5
            msg.percentage = self.pct
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
            self.pct = max(0.0, self.pct - 0.000139)  # ~2hr drain

        msg.charge = self.pct * 5.0
        msg.capacity = 5.0
        msg.design_capacity = 5.0
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        self.pub.publish(msg)

        # Log threshold crossings
        pct_int = int(self.pct * 100)
        if pct_int <= 5 and self.last_threshold != 'critical':
            log(f'CRITICAL — {pct_int}% remaining. Requesting shutdown.')
            self.last_threshold = 'critical'
        elif pct_int <= 10 and self.last_threshold not in ('critical', 'low'):
            log(f'WARNING — {pct_int}% remaining. Charge immediately.')
            self.last_threshold = 'low'
        elif pct_int <= 20 and self.last_threshold not in ('critical', 'low', 'warn'):
            log(f'WARNING — below 20% threshold ({pct_int}%)')
            self.last_threshold = 'warn'


def main():
    rclpy.init()
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
