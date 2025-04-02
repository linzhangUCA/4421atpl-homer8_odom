import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry

from serial import Serial
from tf_transformations import quaternion_from_euler
from math import sin, cos, pi


class Homer8OdomNode(Node):
    def __init__(self):
        super().__init__("homer8_odom_talker_node")
        # Init serial port
        self.pico_messenger = Serial("/dev/ttyACM0", 115200)
        self.pico_interact_timer = self.create_timer(0.05, self.talk_listen_pico)
        self.real_lin_vel = 0.0
        self.real_ang_vel = 0.0
        # # Init \cmd_vel subscriber
        # self.target_vel_subscriber = self.create_subscription(
        #     topic="cmd_vel",
        #     msg_type=Twist,
        #     callback=self.send_pico_target_vel,
        #     qos_profile=1,
        # )
        ### START CODEING HERE ### ~? lines
        # Init \odom publisher and tf broadcaster `odom` -> `base_link`
        self.odom_publisher = self.create_publisher(
            topic="odom",
            msg_type=Odometry,
            qos_profile=1,
        )
        self.odom_base_broadcaster = TransformBroadcaster(self)
        self.odom_talker_timer = self.create_timer(0.02, self.announce_odometry)  # 50Hz
        ### END CODING HERE ###
        self.x = 0.0  # in odom frame
        self.y = 0.0
        self.z = 0.033
        self.th = 0.0
        self.prev_ts = self.get_clock().now()
        self.curr_ts = self.get_clock().now()
        self.circle_counter = 0
        self.reverse_circle = False
        self.get_logger().info("HomeR is awaken.")

    def talk_listen_pico(self):
        # Talk target velocity to pico
        if self.circle_counter <= 80:
            targ_lin_vel = pi / 8
            targ_ang_vel = -pi / 2
        elif 80 < self.circle_counter <= 160:
            targ_lin_vel = pi / 7
            targ_ang_vel = pi / 2
        elif self.circle_counter > 160:
            targ_lin_vel = 0.0
            targ_ang_vel = 0.0
        self.pico_messenger.write(f"{targ_lin_vel},{targ_ang_vel}\n".encode("utf-8"))
        self.circle_counter += 1
        self.get_logger().info(
            f"target velocity set \n---\nlinear: {targ_lin_vel}, angular: {targ_ang_vel}"
        )
        # Listen real velocity from pico
        if self.pico_messenger.inWaiting() > 0:
            real_vels = (
                self.pico_messenger.readline().decode("utf-8").rstrip().split(",")
            )
            if len(real_vels) == 2:
                self.real_lin_vel = float(real_vels[0])
                self.real_ang_vel = float(real_vels[1])
                self.get_logger().info(
                    f"actual velocity measured \n---\nlinear: {self.real_lin_vel}, angular: {self.real_ang_vel}"
                )
            else:
                self.get_logger().warn("Pico message decoding ABNORMAL!")

    # def send_pico_target_vel(self, cmd_vel_msg):
    #     targ_linv = cmd_vel_msg.linear.x
    #     targ_angv = cmd_vel_msg.angular.z
    #     self.pico_messenger.write(f"{targ_linv},{targ_angv}\n".encode("utf-8"))
    #     self.get_logger().info(
    #         f"target velocity set \n---\nlinear: {targ_linv}, angular: {targ_angv}"
    #     )

    def announce_odometry(self):
        # Calculate time and pose change
        self.curr_ts = self.get_clock().now()
        dt = (self.curr_ts - self.prev_ts).nanoseconds * 1e-9
        dx = self.real_lin_vel * cos(self.th) * dt
        dy = self.real_lin_vel * sin(self.th) * dt
        dth = self.real_ang_vel * dt
        self.x += dx
        self.y += dy
        self.th += dth
        quat = quaternion_from_euler(0, 0, self.th)
        self.prev_ts = self.curr_ts  # update time stamp
        # broadcast `odom` to `base_link` transformation
        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = self.curr_ts.to_msg()
        odom_base_tf.header.frame_id = "odom"
        odom_base_tf.child_frame_id = "base_link"
        odom_base_tf.transform.translation.x = self.x
        odom_base_tf.transform.translation.y = self.y
        odom_base_tf.transform.translation.z = self.z
        odom_base_tf.transform.rotation.x = quat[0]
        odom_base_tf.transform.rotation.y = quat[1]
        odom_base_tf.transform.rotation.z = quat[2]
        odom_base_tf.transform.rotation.w = quat[3]
        self.odom_base_broadcaster.sendTransform(odom_base_tf)
        # publish odom topic
        odom_msg = Odometry()
        odom_msg.header.stamp = self.curr_ts.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = self.real_lin_vel
        odom_msg.twist.twist.angular.z = self.real_ang_vel
        self.odom_publisher.publish(odom_msg)


def main():
    rclpy.init()
    homer8_node = Homer8OdomNode()
    rclpy.spin(homer8_node)
    homer8_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
