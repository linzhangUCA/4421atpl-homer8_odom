import rclpy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf_transformations import quaternion_from_euler
from math import sin, cos

from homer8_odom_pkg.homer_figure8 import Homer8Node


class OdomAnnouncerNode(Homer8Node):
    def __init__(self):
        super().__init__()
        ### START CODEING HERE ### ~3 lines
        # Init /odom publisher and tf broadcaster `odom` -> `base_link`
        self.odom_base_broadcaster = None
        self.odom_publisher = None
        self.odom_talker_timer = None  # 50Hz
        ### END CODING HERE ###
        # Init variables
        self.x = 0.0  # in odom frame
        self.y = 0.0
        self.z = 0.033
        self.th = 0.0
        self.prev_ts = self.get_clock().now()
        self.curr_ts = self.get_clock().now()
        self.get_logger().info("Odometry is ready.")

    def announce_odometry(self):
        """Callback function for odom_talker_timer
        Totally 3 steps:
            1. Calculate pose change at every instance.
            2. Broadcast transformation: "odom" -> "base_link".
            3. Publish "/odom" topic.
        """

        ### START CODEING HERE ### ~30 lines

        # Calculate time and pose change
        self.curr_ts = None
        dt = None
        dx = None
        dy = None
        dth = None
        self.x = None
        self.y = None
        self.th = None
        quat = quaternion_from_euler(None)
        self.prev_ts = self.curr_ts  # update previous time stamp

        # Broadcast `odom` to `base_link` transformation
        odom_base_tf = TransformStamped()
        odom_base_tf.header.stamp = None
        odom_base_tf.header.frame_id = None
        odom_base_tf.child_frame_id = None
        odom_base_tf.transform.translation.x = None
        odom_base_tf.transform.translation.y = None
        odom_base_tf.transform.translation.z = None
        odom_base_tf.transform.rotation.x = None
        odom_base_tf.transform.rotation.y = None
        odom_base_tf.transform.rotation.z = None
        odom_base_tf.transform.rotation.w = None
        self.odom_base_broadcaster.sendTransform(odom_base_tf)  # broadcast tf

        # publish odom topic
        odom_msg = Odometry()
        odom_msg.header.stamp = None
        odom_msg.header.frame_id = None
        odom_msg.child_frame_id = None
        odom_msg.pose.pose.position.x = None
        odom_msg.pose.pose.position.y = None
        odom_msg.pose.pose.position.z = None
        odom_msg.pose.pose.orientation.x = None
        odom_msg.pose.pose.orientation.y = None
        odom_msg.pose.pose.orientation.z = None
        odom_msg.pose.pose.orientation.w = None
        odom_msg.twist.twist.linear.x = self.real_lin_vel
        odom_msg.twist.twist.angular.z = self.real_ang_vel
        self.odom_publisher.publish(odom_msg)  # publish "/odom" topic
        ### END CODING HERE ###


def main():
    rclpy.init()
    odom_node = OdomAnnouncerNode()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
