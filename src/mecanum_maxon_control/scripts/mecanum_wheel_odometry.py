#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MecanumWheelOdometry(Node):
    """Integrates wheel odometry from the four EPOS4 motor encoders."""

    def __init__(self):
        super().__init__("mecanum_wheel_odometry")
        self.declare_parameter("wheel_radius", 0.10)
        self.declare_parameter("wheelbase", 0.54)
        self.declare_parameter("track_width", 0.58)
        self.declare_parameter("gear_ratio", 21.0)
        self.declare_parameter("motor_directions", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("odom_topic", "odometry/wheel")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter(
            "twist_covariance_diagonal", [0.01, 0.01, 0.0, 0.0, 0.0, 0.02]
        )
        self.declare_parameter(
            "pose_covariance_diagonal", [0.01, 0.01, 0.0, 0.0, 0.0, 0.02]
        )

        self.radius = float(self.get_parameter("wheel_radius").value)
        self.k = 0.5 * (
            float(self.get_parameter("wheelbase").value)
            + float(self.get_parameter("track_width").value)
        )
        self.gear_ratio = float(self.get_parameter("gear_ratio").value)
        self.directions = list(self.get_parameter("motor_directions").value)
        self.rate = float(self.get_parameter("publish_rate").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.twist_cov_diag = list(
            self.get_parameter("twist_covariance_diagonal").value
        )
        self.pose_cov_diag = list(self.get_parameter("pose_covariance_diagonal").value)

        self.wheel_velocity = [0.0] * 4
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        for index in range(1, 5):
            self.create_subscription(
                JointState,
                f"/cia402_device_{index}/joint_states",
                lambda msg, wheel=index - 1: self._joint_state_callback(msg, wheel),
                10,
            )

        self.odom_pub = self.create_publisher(
            Odometry, str(self.get_parameter("odom_topic").value), 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / self.rate, self._update)
        self.get_logger().info("Wheel odometry node started (dead-reckoning only)")

    def _joint_state_callback(self, msg: JointState, wheel: int):
        if not msg.velocity:
            return
        # Feedback is at the motor shaft (same convention as the COTargetDouble
        # velocity target in mecanum_epos4_controller.py) -- undo gear reduction
        # and the mounting-direction sign to recover wheel angular velocity.
        self.wheel_velocity[wheel] = (
            msg.velocity[0] * self.directions[wheel] / self.gear_ratio
        )

    def _update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        w0, w1, w2, w3 = self.wheel_velocity
        # Closed-form inverse of the forward mecanum kinematics in
        # mecanum_epos4_controller.py's cmd_vel_callback (wheel order:
        # front-left, front-right, rear-left, rear-right).
        vx = self.radius / 4.0 * (w0 + w1 + w2 + w3)
        vy = self.radius / 4.0 * (-w0 + w1 + w2 - w3)
        wz = self.radius / (4.0 * self.k) * (-w0 + w1 - w2 + w3)

        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        self.x += delta_x
        self.y += delta_y
        self.theta = math.atan2(
            math.sin(self.theta + wz * dt), math.cos(self.theta + wz * dt)
        )

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self._expand_covariance(self.pose_cov_diag)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = self._expand_covariance(self.twist_cov_diag)
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame_id
            tf_msg.child_frame_id = self.base_frame_id
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

    @staticmethod
    def _expand_covariance(diagonal):
        matrix = [0.0] * 36
        for i, value in enumerate(diagonal):
            matrix[i * 6 + i] = value
        return matrix


def main(args=None):
    rclpy.init(args=args)
    node = MecanumWheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
