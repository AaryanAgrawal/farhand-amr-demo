#!/usr/bin/env python3
"""Simulated battery monitor — publishes BatteryState.

Fault: fault.critical → voltage 10.2V, 3%, NOT_CHARGING, jitter when <5%.
"""

import os
import random
from datetime import datetime

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import BatteryState

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

        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('initial_pct', 0.85)
        self.declare_parameter('capacity_ah', 5.0)
        self.declare_parameter('fault.critical', False)

        rate = self.get_parameter('rate_hz').value
        self.capacity = self.get_parameter('capacity_ah').value
        self.critical = self.get_parameter('fault.critical').value
        initial_pct = self.get_parameter('initial_pct').value

        self.pct = 0.03 if self.critical else initial_pct
        self.last_threshold = None
        self.pub = self.create_publisher(BatteryState, 'battery_state', 10)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.add_on_set_parameters_callback(self._on_param_change)

        mode = 'CRITICAL (10.2V, 3%)' if self.critical else f'normal ({int(self.pct * 100)}%)'
        self.get_logger().info(f'Battery monitor started ({rate}Hz, {mode})')
        log(f'Node started. Mode: {mode}')

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.critical':
                self.critical = p.value
                if self.critical:
                    self.pct = 0.03
                    log('FAULT INJECTED — critical mode active')
                else:
                    self.pct = self.get_parameter('initial_pct').value
                    log('Fault cleared — normal mode restored')
        return SetParametersResult(successful=True)

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
            self.pct = max(0.0, self.pct - 0.000139)

        msg.charge = self.pct * self.capacity
        msg.capacity = self.capacity
        msg.design_capacity = self.capacity
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        self.pub.publish(msg)

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
