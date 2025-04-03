import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from serial import Serial
from math import pi


class Homer8Node(Node):
    def __init__(self):
        super().__init__(node_name="homer_figure8_node")
        # Init serial port
        self.pico_messenger = Serial("/dev/ttyACM0", 115200)
        self.pico_interact_timer = self.create_timer(0.02, self.talk_listen_pico)
        self.real_lin_vel = 0.0
        self.real_ang_vel = 0.0
        # Init turtle1/cmd_vel publisher
        self.turtle_cmd_publisher = self.create_publisher(Twist, "turtle1/cmd_vel", 1)
        self.circle_counter = 0
        self.get_logger().info("HomeR is awaken.")

    def talk_listen_pico(self):
        # Talk target velocity to pico
        if 300 < self.circle_counter <= 500:
            targ_lin_vel = pi / 8
            targ_ang_vel = -pi / 2
        elif 500 < self.circle_counter <= 750:
            targ_lin_vel = pi / 7
            targ_ang_vel = pi / 2
        else:
            targ_lin_vel = 0.0
            targ_ang_vel = 0.0
        self.pico_messenger.write(f"{targ_lin_vel},{targ_ang_vel}\n".encode("utf-8"))
        self.circle_counter += 1
        self.get_logger().debug(
            f"target velocity set \n---\nlinear: {targ_lin_vel}, angular: {targ_ang_vel}"
        )
        # Listen real velocity from pico
        if self.pico_messenger.inWaiting() > 0:
            real_vels = (
                self.pico_messenger.readline().decode("utf-8").rstrip().split(",")
            )
            if len(real_vels) == 2:
                try:
                    (self.real_lin_vel, self.real_ang_vel) = tuple(
                        map(float, real_vels)
                    )
                except ValueError:
                    pass
                self.get_logger().debug(
                    f"actual velocity measured \n---\nlinear: {self.real_lin_vel}, angular: {self.real_ang_vel}"
                )
                # Drive turtle1
                twist_msg = Twist()
                twist_msg.linear.x = self.real_lin_vel * 4
                twist_msg.angular.z = self.real_ang_vel
                self.turtle_cmd_publisher.publish(twist_msg)
                self.get_logger().debug(f"velocity command for turtle: \n{twist_msg}")
            else:
                self.get_logger().warn("Pico message decoding ABNORMAL!")


def main():
    rclpy.init()
    homer8_node = Homer8Node()
    rclpy.spin(homer8_node)
    homer8_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
