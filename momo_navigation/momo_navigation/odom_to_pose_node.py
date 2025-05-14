import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomToPoseNode(Node):
    def __init__(self):
        super().__init__("odom_to_pose_node")

        self.odom_subscriber = self.create_subscription(
            Odometry, "/momo/odom", self.odom_callback, 10
        )

        self.pose_publisher = self.create_publisher(Pose, "/momo/pose", 10)

        self.get_logger().info("OdomToPoseNode has been started.")

    def odom_callback(self, msg: Odometry):
        pose_msg = msg.pose.pose
        self.pose_publisher.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
