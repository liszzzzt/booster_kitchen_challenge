import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COMMAND_TIMEOUT_SECONDS = 10.0


class T2ServerNode(Node):
    def __init__(self):
        super().__init__('t2_server_node')

        # command_id: state
        self.command_status = {}

        self.locomotion_status_subscription = self.create_subscription(String, "locomotion_status", self.locomotion_status_callback, 10)
        self.manipulation_status_subscription = self.create_subscription(String, "manipulation_status", self.manipulation_status_callback, 10)

        self.locomotion_command_publisher = self.create_publisher(
            String, "locomotion_command", 10
        )
        self.manipulation_command_publisher = self.create_publisher(
            String, "manipulation_command", 10
        )

    def wait_for_success(self, command_id):
        deadline = time.monotonic() + COMMAND_TIMEOUT_SECONDS

        while rclpy.ok():
            state = self.command_status.get(command_id)
            if state == "SUCCESS":
                return True
            if state == "FAIL":
                return False
            if time.monotonic() >= deadline:
                return False

            time.sleep(0.1)

        return False

    def locomotion_status_callback(self, msg):
        status = json.loads(msg.data)
        command_id = status["id"]
        state = status["state"]
        self.command_status[command_id] = state

    def manipulation_status_callback(self, msg):
        status = json.loads(msg.data)
        command_id = status["id"]
        state = status["state"]
        self.command_status[command_id] = state

    def send_locomotion_command(self, command):
        msg = String()
        msg.data = json.dumps(command)
        self.command_status[command["id"]] = "PENDING"
        self.locomotion_command_publisher.publish(msg)

    def send_manipulation_command(self, command):
        msg = String()
        msg.data = json.dumps(command)
        self.command_status[command["id"]] = "PENDING"
        self.manipulation_command_publisher.publish(msg)
