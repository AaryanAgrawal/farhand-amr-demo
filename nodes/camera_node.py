#!/usr/bin/env python3
"""Simulated USB camera — publishes Image + CameraInfo at 30Hz."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np


class CameraSimNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish)  # 30 Hz
        self.frame = 0
        self.get_logger().info('Camera simulator started (30Hz, 640x480)')

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        w, h = 640, 480

        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:, :, 0] = np.arange(w, dtype=np.uint8)
        img[:, :, 1] = np.arange(h, dtype=np.uint8).reshape(-1, 1)
        img[:, :, 2] = self.frame % 256

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_link'
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = img.tobytes()
        self.img_pub.publish(msg)

        info = CameraInfo()
        info.header = msg.header
        info.height = h
        info.width = w
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = fy = 500.0
        info.k = [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

        self.frame += 1


def main():
    rclpy.init()
    node = CameraSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
