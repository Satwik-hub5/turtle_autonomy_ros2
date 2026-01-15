import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math


class TurtleOdometry(Node):

    def __init__(self):
        super().__init__('turtle_odometry')

        # Publisher for odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/turtle1/odom',
            10
        )

        # Subscriber to turtle pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish static TF ONCE
        self.publish_static_tf()

    def publish_static_tf(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = 'map'
        static_tf.child_frame_id = 'odom'

        static_tf.transform.translation.x = 5.5
        static_tf.transform.translation.y = 5.5
        static_tf.transform.translation.z = 0.0

        static_tf.transform.rotation.x = 0.0
        static_tf.transform.rotation.y = 0.0
        static_tf.transform.rotation.z = 0.0
        static_tf.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(static_tf)
        self.get_logger().info('Static TF map -> odom published')

    def pose_callback(self, msg):
        # ---------------- ODOMETRY ----------------
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(msg.theta / 2.0)
        qw = math.cos(msg.theta / 2.0)

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)

        # ---------------- TF ----------------
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'

        tf.transform.translation.x = msg.x
        tf.transform.translation.y = msg.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
