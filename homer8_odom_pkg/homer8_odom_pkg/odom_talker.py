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
        self.real_lin_vel = 0.0
        self.real_ang_vel = 0.0
        # Config publishers and subscribers

    def receive_pico_message(self):
        if self.pico_messenger.inWaiting() > 0:
            real_vels = (
                self.pico_messenger.readline().decode("utf-8").rstrip().split(",")
            )
            if len(real_vels) == 2:
                self.real_lin_vel = real_vels[0]
                self.real_ang_vel = real_vels[1]
            self.get_logger().debug(
                f"actual velocity measured \n---\nlinear: {self.real_lin_vel}, angular: {self.real_ang_vel}"
            )


def main():
    rclpy.init()
    homer8_node = Homer8OdomNode()
    rclpy.spin(homer8_node)
    homer8_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
