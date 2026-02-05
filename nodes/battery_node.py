#!/usr/bin/env python3
"""Simulated battery monitor — publishes BatteryState at 1Hz."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class BatterySimNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        self.pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0, self.publish)  # 1 Hz
        self.pct = 0.85  # Start at 85%
        self.get_logger().info('Battery monitor started (1Hz, 85% charge)')

    def publish(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 11.1 + (self.pct * 1.5)
        msg.current = -1.5
        msg.charge = self.pct * 5.0
        msg.capacity = 5.0
        msg.design_capacity = 5.0
        msg.percentage = self.pct
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        msg.present = True
        self.pub.publish(msg)
        self.pct = max(0.0, self.pct - 0.000139)  # ~2hr drain


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
