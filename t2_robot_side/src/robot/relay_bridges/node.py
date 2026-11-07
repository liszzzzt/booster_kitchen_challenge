from rclpy.node import Node


class T2RelayNode(Node):
    def __init__(self):
        super().__init__("t2_relay_bridges")
