#!/usr/bin/env python3
"""Simulated payload/gripper — publishes payload status at 1Hz, gripper state at 2Hz.

Fault mode: GRIPPER_FAULT=1 → gripper stuck in "moving", weight_kg = -1.0 (sensor disconnected).
"""

import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime

LOG_PATH = os.path.expanduser('~/logs/payload.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] PAYLOAD: {msg}\n')
    except Exception:
        pass


class PayloadNode(Node):
    def __init__(self):
        super().__init__('payload_node')
        self.status_pub = self.create_publisher(String, '/bcr_bot/payload/status', 10)
        self.gripper_pub = self.create_publisher(String, '/bcr_bot/payload/gripper_state', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.gripper_timer = self.create_timer(0.5, self.publish_gripper)

        self.fault = os.environ.get('GRIPPER_FAULT') == '1'
        self.loaded = not self.fault  # If fault, we were picking up
        self.weight_kg = -1.0 if self.fault else 22.5
        self.pallet_id = 'PLT-2026-0847'
        self.gripper_state = 'moving' if self.fault else 'closed'

        mode = 'FAULT (gripper stuck, weight sensor disconnected)' if self.fault else 'normal'
        self.get_logger().info(f'Payload node started ({mode})')

        if self.fault:
            log('ERROR — Gripper stuck in MOVING state for >10s')
            log('ERROR — Weight sensor reading -1.0kg (disconnected?)')
            log(f'WARNING — Pallet {self.pallet_id} may be unsecured at dock_station_3')
        else:
            log(f'Gripper CLOSED. Weight: {self.weight_kg}kg. Pallet: {self.pallet_id}')

    def publish_status(self):
        status = {
            'loaded': self.loaded and self.weight_kg > 0,
            'weight_kg': self.weight_kg,
            'pallet_id': self.pallet_id if self.loaded else None,
            'dock_station': 'DS-03',
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_gripper(self):
        msg = String()
        msg.data = self.gripper_state
        self.gripper_pub.publish(msg)


def main():
    rclpy.init()
    node = PayloadNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
