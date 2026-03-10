#!/usr/bin/env python3
"""Simulated differential drive — subscribes cmd_vel, publishes odom + joint_states.

Fault: fault.motor_overcurrent → zero velocities, motor fault code 0x03 logged.
"""

import os
import math
from datetime import datetime

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

LOG_PATH = os.path.expanduser('~/logs/drive.log')


def log(msg):
    ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    try:
        with open(LOG_PATH, 'a') as f:
            f.write(f'[{ts}] DRIVE: {msg}\n')
    except Exception:
        pass


class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('wheel_separation', 0.6)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('fault.motor_overcurrent', False)

        rate = self.get_parameter('rate_hz').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_rad = self.get_parameter('wheel_radius').value
        self.fault = self.get_parameter('fault.motor_overcurrent').value

        self.dt = 1.0 / rate
        self.x = self.y = self.theta = 0.0
        self.vx = self.wz = 0.0
        self.left_pos = self.right_pos = 0.0
        self.fault_logged = False

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(self.dt, self.update)

        self.add_on_set_parameters_callback(self._on_param_change)

        mode = 'FAULT (motor 0x03)' if self.fault else 'normal'
        self.get_logger().info(f'Differential drive started ({rate}Hz, {mode})')
        log(f'Node started. Mode: {mode}')
        self._log_fault_if_needed()

    def _log_fault_if_needed(self):
        if self.fault and not self.fault_logged:
            log('ERROR — Motor controller fault code 0x03 (overcurrent left motor)')
            log('Emergency stop engaged. Velocity commands ignored.')
            log('Attempting auto-recovery... FAILED')
            log('CRITICAL — Persistent fault. Manual intervention required.')
            self.fault_logged = True

    def _on_param_change(self, params):
        for p in params:
            if p.name == 'fault.motor_overcurrent':
                self.fault = p.value
                if self.fault:
                    self.fault_logged = False
                    self._log_fault_if_needed()
                    self.vx = self.wz = 0.0
                else:
                    log('Fault cleared — motor control restored')
                    self.fault_logged = False
        return SetParametersResult(successful=True)

    def cmd_vel_cb(self, msg):
        if not self.fault:
            self.vx = msg.linear.x
            self.wz = msg.angular.z

    def update(self):
        if self.fault:
            vl = vr = 0.0
        else:
            self.x += self.vx * math.cos(self.theta) * self.dt
            self.y += self.vx * math.sin(self.theta) * self.dt
            self.theta += self.wz * self.dt
            vl = self.vx - (self.wheel_sep / 2.0) * self.wz
            vr = self.vx + (self.wheel_sep / 2.0) * self.wz
            self.left_pos += (vl / self.wheel_rad) * self.dt + 0.001  # encoder drift
            self.right_pos += (vr / self.wheel_rad) * self.dt

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = 0.0 if self.fault else self.vx
        odom.twist.twist.angular.z = 0.0 if self.fault else self.wz
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = now
        js.name = ['middle_left_wheel_joint', 'middle_right_wheel_joint']
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [0.0, 0.0] if self.fault else [vl / self.wheel_rad, vr / self.wheel_rad]
        js.effort = [0.0, 0.0]
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = DiffDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
