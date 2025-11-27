#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sys
import termios
import tty
import select


def get_key():
    """Non-blocking keyboard input (Linux)."""
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')

        self.publisher_ = self.create_publisher(Bool, '/next_waypoint', 10)

        self.get_logger().info("Keyboard Controller Started!")
        self.get_logger().info("Press SPACE or ENTER → Move to next waypoint")
        self.get_logger().info("Press q → Quit")

        # Timer to check keyboard input
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        key = get_key()
        if key:
            if key == ' ' or key == '\n':  # Space or Enter
                msg = Bool()
                msg.data = True
                self.publisher_.publish(msg)
                self.get_logger().info("Next waypoint triggered!")

            elif key == 'q':
                self.get_logger().info("Exiting keyboard controller...")
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
