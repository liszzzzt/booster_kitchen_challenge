from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuRelay:
    def __init__(self, node):
        self.publisher = node.create_publisher(
            Imu,
            "/t2/imu/data",
            qos_profile_sensor_data,
        )
        self.subscription = node.create_subscription(
            Imu,
            "/imu/data",
            self.publisher.publish,
            qos_profile_sensor_data,
        )
