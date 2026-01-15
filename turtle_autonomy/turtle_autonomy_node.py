import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, PoseStamped
import math

# ---------------- Predictive parameters ----------------
yaw_options = [-1.0, -0.5, 0.0, 0.5, 1.0]   # possible angular velocities (rad/s)
dt = 0.1  # time step for prediction
v = 1.0   # constant forward speed
prediction_steps = 10  # how many steps ahead to predict

# ---------------- Turtle Autonomy Node ----------------
class TurtleAutonomy(Node):

    def __init__(self):
        super().__init__('turtle_autonomy')

        self.current_pose = None
        self.goal_pose = None

        # Subscribe to turtle pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Subscribe to goal
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    # ---------------- Callbacks ----------------
    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.goal_pose = msg.pose

    # ---------------- Predict future position ----------------
    def predict_position(self, x, y, theta, omega):
        """Predict future position after several steps with given angular velocity"""
        for _ in range(prediction_steps):
            theta += omega * dt
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
        return x, y

    # ---------------- Main control loop ----------------
    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            return

        # Compute distance to goal
        dx_goal = self.goal_pose.position.x - self.current_pose.x
        dy_goal = self.goal_pose.position.y - self.current_pose.y
        distance_to_goal = math.sqrt(dx_goal**2 + dy_goal**2)

        # Stop if close enough
        cmd = Twist()
        if distance_to_goal < 0.1:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # ---------------- Predictive logic ----------------
        best_yaw = 0.0
        min_distance = float('inf')

        for yaw in yaw_options:
            x_pred, y_pred = self.predict_position(
                self.current_pose.x,
                self.current_pose.y,
                self.current_pose.theta,
                yaw
            )
            dx = self.goal_pose.position.x - x_pred
            dy = self.goal_pose.position.y - y_pred
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_distance:
                min_distance = dist
                best_yaw = yaw

        # ---------------- Send command ----------------
        cmd.linear.x = v
        cmd.angular.z = best_yaw
        self.cmd_pub.publish(cmd)


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleAutonomy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
