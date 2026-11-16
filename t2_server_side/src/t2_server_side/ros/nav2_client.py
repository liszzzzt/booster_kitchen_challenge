import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class T2Nav2Client(Node):
    def __init__(self):
        super().__init__("t2_nav2_client")
        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )

    def navigate_to(self, x, y, yaw):
        while rclpy.ok() and not self.client.wait_for_server(timeout_sec=1.0):
            print("Waiting for Nav2...")

        if not rclpy.ok():
            return False

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        print(f"Nav2 goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        send_future = self.client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("Nav2 goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)

        if not rclpy.ok():
            return False

        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            print("Nav2 goal completed")
            return True

        print(f"Nav2 failed with status {result.status}")
        return False
