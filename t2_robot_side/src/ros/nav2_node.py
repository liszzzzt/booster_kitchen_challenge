import time

from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class T2Nav2Node(Node):
    COMMAND_TIMEOUT = 0.5

    def __init__(self, t2):
        super().__init__("t2_nav2")
        self.t2 = t2
        self.last_command_time = None
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_subscription = self.create_subscription(
            TwistStamped,
            "cmd_vel",
            self.command_callback,
            command_qos,
        )
        self.watchdog = self.create_timer(0.1, self.watchdog_callback)

    def command_callback(self, msg):
        self.last_command_time = time.monotonic()
        self.t2.locomotion(msg, source="nav2")

    def watchdog_callback(self):
        if (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time > self.COMMAND_TIMEOUT
        ):
            self.t2.stop_nav2()
            self.last_command_time = None

