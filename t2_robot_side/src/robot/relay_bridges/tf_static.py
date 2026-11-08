from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


class TfStaticRelay:
    def __init__(self, node):
        self.transforms = {}
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = node.create_publisher(TFMessage, "/t2/tf_static", qos)
        self.subscription = node.create_subscription(
            TFMessage,
            "/tf_static",
            self._relay,
            qos,
        )

    def _relay(self, msg):
        for transform in msg.transforms:
            self.transforms[transform.child_frame_id] = transform

        relayed = TFMessage()
        relayed.transforms = list(self.transforms.values())
        self.publisher.publish(relayed)
