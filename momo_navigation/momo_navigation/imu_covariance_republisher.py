import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMURepublisher(Node):
    def __init__(self):
        super().__init__('imu_covariance_republisher')
        self.sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pub = self.create_publisher(Imu, '/imu_fixed', 10)

    def imu_callback(self, msg: Imu):
        # Set small but realistic covariances
        msg.orientation_covariance = [0.05, 0.0, 0.0,
                                      0.0, 0.05, 0.0,
                                      0.0, 0.0, 0.05]
        msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                           0.0, 0.01, 0.0,
                                           0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                              0.0, 0.1, 0.0,
                                              0.0, 0.0, 0.1]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = IMURepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
