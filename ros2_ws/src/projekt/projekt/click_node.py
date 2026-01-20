#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class ClickDriveNode(Node):
    def __init__(self):
        super().__init__('click_node')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.15)

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)

        self.bridge = CvBridge()
        self.last_frame = None

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.window_name = 'click_node'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.timer = self.create_timer(0.03, self.on_gui_tick)

        self.get_logger().info(
            f"Subscribed: {self.image_topic} | Publishing: {self.cmd_vel_topic} | v={self.linear_speed}"
        )

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.last_frame = frame

    def on_gui_tick(self):
        if self.last_frame is None:
            cv2.waitKey(1)
            return

        h, w = self.last_frame.shape[:2]
        mid_y = h // 2

        display = self.last_frame.copy()
        cv2.line(display, (0, mid_y), (w, mid_y), (255, 255, 255), 1)
        cv2.imshow(self.window_name, display)
        cv2.waitKey(1)

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self.last_frame is None:
            return

        h = self.last_frame.shape[0]
        mid_y = h // 2

        msg = Twist()
        if y < mid_y:
            msg.linear.x = self.linear_speed
        else:
            msg.linear.x = -self.linear_speed

        self.pub.publish(msg)
        self.get_logger().info(f"Click y={y} mid={mid_y} -> linear.x={msg.linear.x}")


def main(args=None):
    rclpy.init(args=args)
    node = ClickDriveNode()
    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
