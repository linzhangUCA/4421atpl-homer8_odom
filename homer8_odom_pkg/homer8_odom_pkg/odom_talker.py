from serial import Serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from math import pi


class Homer8OdomNode(Node):
    def __init__(self):
        super().__init__("homer8_odom_talker_node")
        # Config serial port
        self.pico_messenger = Serial("/dev/ttyACM0", 115200)
        self.pico_receiver_timer = self.create_timer(0.015, self.receive_pico_message)
        # Config publishers and subscribers

    def receive_pico_message(self):
        if self.pico_messenger.in_waiting() > 0:
            real_vel = (
                self.pico_messenger.readline().decode("urf-8").rstrip().split(",")
            )
        self.get_logger().info(f"HomeR's actual velocity measured as: {real_vel}.")


def main():
    rclpy.init()
    homer8_node = Homer8OdomNode()
    rclpy.spin(homer8_node)
    homer8_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
