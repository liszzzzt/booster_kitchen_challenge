from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStatesRelay:
    def __init__(self, node):
        self.publisher = node.create_publisher(
            JointState,
            "/t2/joint_states",
            qos_profile_sensor_data,
        )
        self.subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self.publisher.publish,
            qos_profile_sensor_data,
        )
