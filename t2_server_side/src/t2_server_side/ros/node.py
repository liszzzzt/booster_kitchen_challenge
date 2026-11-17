import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String

COMMAND_TIMEOUT_SECONDS = 15.0
DDS_CONNECTION_TIMEOUT_SECONDS = 15.0
DOOR_CHECK_TIMEOUT_SECONDS = 5.0

# check door open | door closed, at the beginning
DOOR_X_MIN = 0.75
DOOR_X_MAX = 1.15
DOOR_Y_MIN = -0.25
DOOR_Y_MAX = 0.25
DOOR_Z_MIN = -0.50
DOOR_Z_MAX = 0.50
DOOR_CLOSED_POINT_THRESHOLD = 1000
DOOR_CHECK_SAMPLES = 3


class T2ServerNode(Node):
    def __init__(self):
        super().__init__('t2_server_node')

        # command_id: state
        self.command_status = {}
        self.door_results = []

        self.locomotion_status_subscription = self.create_subscription(String, "locomotion_status", self.locomotion_status_callback, 10)
        self.manipulation_status_subscription = self.create_subscription(String, "manipulation_status", self.manipulation_status_callback, 10)
        self.door_subscription = self.create_subscription(
            PointCloud2,
            "/e1r/points",
            self.door_callback,
            qos_profile_sensor_data,
        )

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

    def door_callback(self, msg):
        points = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        )
        door_points = (
            (points[:, 0] > DOOR_X_MIN)
            & (points[:, 0] < DOOR_X_MAX)
            & (points[:, 1] > DOOR_Y_MIN)
            & (points[:, 1] < DOOR_Y_MAX)
            & (points[:, 2] > DOOR_Z_MIN)
            & (points[:, 2] < DOOR_Z_MAX)
        )
        point_count = int(np.count_nonzero(door_points))
        self.door_results.append(point_count)
        self.door_results = self.door_results[-DOOR_CHECK_SAMPLES:]

    def check_door(self):
        deadline = time.monotonic() + DOOR_CHECK_TIMEOUT_SECONDS

        while rclpy.ok():
            if len(self.door_results) == DOOR_CHECK_SAMPLES:
                counts = self.door_results.copy()
                door_open = all(
                    count < DOOR_CLOSED_POINT_THRESHOLD for count in counts
                )
                self.destroy_subscription(self.door_subscription)
                self.door_subscription = None
                state = "OPEN" if door_open else "CLOSED"
                print(f"Door check: {counts} points - {state}")
                return door_open

            if time.monotonic() >= deadline:
                self.destroy_subscription(self.door_subscription)
                self.door_subscription = None
                print("Door check failed: no E1R point cloud received.")
                return False

            time.sleep(0.1)

        return False

    # initial DDS ready check
    def wait_for_robot(self):
        deadline = time.monotonic() + DDS_CONNECTION_TIMEOUT_SECONDS

        while rclpy.ok():
            commands_ready = (
                    self.locomotion_command_publisher.get_subscription_count() > 0
                    and self.manipulation_command_publisher.get_subscription_count() > 0
            )

            statuses_ready = (
                    self.locomotion_status_subscription.get_publisher_count() > 0
                    and self.manipulation_status_subscription.get_publisher_count() > 0
            )

            if commands_ready and statuses_ready:
                return True

            if time.monotonic() >= deadline:
                return False

            time.sleep(0.1)

        return False
