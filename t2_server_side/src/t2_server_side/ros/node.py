from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class T2ServerNode(Node):
    def __init__(self):
        super().__init__("t2_server_side")
        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile_sensor_data,
        )

    def odom_callback(self, message):
        self.get_logger().info("Connected")