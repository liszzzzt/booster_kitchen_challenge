import math

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from tf2_msgs.msg import TFMessage


MAX_TRANSLATION_JUMP = 0.30
MAX_YAW_JUMP = math.radians(20)


class TfJumpFilterNode(Node):
    def __init__(self):
        super().__init__("t2_tf_jump_filter")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.previous_map_to_odom = None
        self.rejecting_jump = False
        self.publisher = self.create_publisher(TFMessage, "/t2/tf_filtered", qos)
        self.subscription = self.create_subscription(
            TFMessage,
            "/t2/tf",
            self.filter_tf,
            qos,
        )

    def get_yaw(self, rotation):
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    def filter_tf(self, msg):
        filtered_transforms = []

        for transform in msg.transforms:
            if transform.header.frame_id != "map" or transform.child_frame_id != "odom":
                filtered_transforms.append(transform)
                continue

            if self.previous_map_to_odom is None:
                self.previous_map_to_odom = transform
                filtered_transforms.append(transform)
                continue

            previous = self.previous_map_to_odom.transform
            current = transform.transform
            translation_jump = math.hypot(
                current.translation.x - previous.translation.x,
                current.translation.y - previous.translation.y,
            )
            yaw_difference = self.get_yaw(current.rotation) - self.get_yaw(
                previous.rotation
            )
            yaw_jump = abs(math.atan2(math.sin(yaw_difference), math.cos(yaw_difference)))

            if (
                translation_jump > MAX_TRANSLATION_JUMP
                or yaw_jump > MAX_YAW_JUMP
            ):
                if not self.rejecting_jump:
                    self.get_logger().warning(
                        "Rejected map -> odom jump: "
                        f"translation={translation_jump:.2f} m, "
                        f"yaw={math.degrees(yaw_jump):.1f} deg"
                    )
                self.rejecting_jump = True
                continue

            if self.rejecting_jump:
                self.get_logger().info("map -> odom transform recovered")

            self.rejecting_jump = False
            self.previous_map_to_odom = transform
            filtered_transforms.append(transform)

        if filtered_transforms:
            self.publisher.publish(TFMessage(transforms=filtered_transforms))
