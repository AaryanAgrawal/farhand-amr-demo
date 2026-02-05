#!/usr/bin/env python3
"""Simulated differential drive — subscribes /cmd_vel, publishes /odom + /joint_states at 50Hz."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math


class DriveSimNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.wheel_sep = 0.34
        self.wheel_rad = 0.05
        self.x = self.y = self.theta = 0.0
        self.vx = self.wz = 0.0
        self.left_pos = self.right_pos = 0.0
        self.get_logger().info('Differential drive simulator started (50Hz)')

    def cmd_vel_cb(self, msg):
        self.vx = msg.linear.x
        self.wz = msg.angular.z

    def update(self):
        dt = 0.02
        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt
        self.theta += self.wz * dt

        vl = self.vx - (self.wheel_sep / 2.0) * self.wz
        vr = self.vx + (self.wheel_sep / 2.0) * self.wz
        self.left_pos += (vl / self.wheel_rad) * dt
        self.right_pos += (vr / self.wheel_rad) * dt

        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [vl / self.wheel_rad, vr / self.wheel_rad]
        js.effort = [0.0, 0.0]
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = DriveSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
