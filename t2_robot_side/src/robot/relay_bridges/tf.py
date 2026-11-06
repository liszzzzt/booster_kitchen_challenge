from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


class TfRelay:
    def __init__(self, node):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = node.create_publisher(TFMessage, "/t2/tf", qos)
        self.subscription = node.create_subscription(
            TFMessage,
            "/tf",
            self.publisher.publish,
            qos,
        )
