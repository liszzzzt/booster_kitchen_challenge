import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class T2RobotNode(Node):
    def __init__(self, t2):
        super().__init__("t2_robot_side")
        self.t2 = t2

        # subscriber
        self.locomotion_command_subscription = self.create_subscription(String, "locomotion_command", self.locomotion_command_callback, 10)
        self.manipulation_command_subscription = self.create_subscription(String, "manipulation_command", self.manipulation_command_callback, 10)

        # publisher
        self.locomotion_status_publisher = self.create_publisher(
            String, "locomotion_status", 10
        )
        self.manipulation_status_publisher = self.create_publisher(
            String, "manipulation_status", 10
        )

    def publish_status(self, publisher, command_id, state):
        msg = String()
        msg.data = json.dumps({
            "id": command_id,
            "state": state,
        })
        publisher.publish(msg)

    def locomotion_command_callback(self, msg):
        command_id = None
        try:
            command = json.loads(msg.data)
            command_id = command["id"]

            self.publish_status(
                self.locomotion_status_publisher,
                command_id,
                "RUNNING",
            )

            self.get_logger().info(f"locomotion: message={msg.data}")
            result = self.t2.locomotion(msg)
        except Exception as error:
            self.get_logger().error(f"locomotion failed: {error}")
            if command_id is not None:
                self.publish_status(
                    self.locomotion_status_publisher,
                    command_id,
                    "FAIL",
                )
            return

        self.publish_status(
            self.locomotion_status_publisher,
            command_id,
            "SUCCESS" if result else "FAIL",
        )


    def manipulation_command_callback(self, msg):
        command_id = None
        try:
            command = json.loads(msg.data)
            command_id = command["id"]

            self.publish_status(
                self.manipulation_status_publisher,
                command_id,
                "RUNNING",
            )

            self.get_logger().info(f"manipulation: message={msg.data}")
            result = self.t2.manipulation(msg)
        except Exception as error:
            self.get_logger().error(f"manipulation failed: {error}")
            if command_id is not None:
                self.publish_status(
                    self.manipulation_status_publisher,
                    command_id,
                    "FAIL",
                )
            return

        self.publish_status(
            self.manipulation_status_publisher,
            command_id,
            "SUCCESS" if result else "FAIL",
        )
