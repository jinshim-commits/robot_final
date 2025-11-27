#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner_node')
        self.bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()

        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            String,
            '/qr_code/data',
            10
        )

        self.get_logger().info("QR Scanner Node started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        data, bbox, _ = self.qr_detector.detectAndDecode(frame)
        if data:
            msg = String()
            msg.data = data
            self.publisher_.publish(msg)
            self.get_logger().info(f"QR Detected: {data}")


def main(args=None):
    rclpy.init(args=args)
    node = QRScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

