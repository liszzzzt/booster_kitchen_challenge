import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class T2Nav2Client(Node):
    def __init__(self):
        super().__init__("t2_nav2_client")
        self.client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
        )
        self.spin_client = ActionClient(
            self,
            Spin,
            "/spin",
        )

        # nav2 debug
        self.last_debug_time = 0.0
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

    def command_callback(self, msg):
        current_time = time.monotonic()
        if current_time - self.last_debug_time >= 0.5:
            print(
                "Nav2 cmd: "
                f"forward={msg.twist.linear.x:.2f}, "
                f"lateral={msg.twist.linear.y:.2f}, "
                f"rotation={msg.twist.angular.z:.2f}"
            )
            self.last_debug_time = current_time

    def spin(self):
        while rclpy.ok() and not self.spin_client.wait_for_server(timeout_sec=1.0):
            print("Waiting for Nav2 spin...")

        if not rclpy.ok():
            return False

        goal = Spin.Goal()
        goal.target_yaw = math.radians(360)
        goal.time_allowance.sec = 25

        print("Nav2 spin: angle=360 degrees")
        send_future = self.spin_client.send_goal_async(goal)
        while rclpy.ok() and not send_future.done():
            time.sleep(0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print("Nav2 spin rejected")
            return False

        result_future = goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            time.sleep(0.1)

        if not rclpy.ok():
            return False

        result = result_future.result()
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            print(f"Nav2 spin failed with status {result.status}")
            return False

        print("Nav2 spin completed")
        return True

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
