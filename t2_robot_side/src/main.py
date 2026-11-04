import rclpy
import threading
from booster_agent_framework import AgentBase, AgentFeatures
from rclpy.executors import SingleThreadedExecutor

from .ros.node import T2RobotNode
from .robot.t2 import T2Robot


class T2KitchenAgent(AgentBase):
    def __init__(self):
        super().__init__(AgentFeatures())

        rclpy.init()

        self.t2 = T2Robot()
        self.robot_node = T2RobotNode(self.t2)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.robot_node)
        self.executor_thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.heartbeat_stop_event = threading.Event()
        self.heartbeat_thread = threading.Thread(
            target=self.heartbeat,
            daemon=True,
        )
        self.executor_started = False

    def on_agent_activated(self):
        if not self.executor_started:
            self.executor_thread.start()
            self.heartbeat_thread.start()
            self.executor_started = True
        print("T2KitchenAgent activated", flush=True)
        self.t2.launch()

    def on_agent_close(self):
        self.heartbeat_stop_event.set()
        self.executor.shutdown()
        if self.executor_started:
            self.executor_thread.join()
            self.heartbeat_thread.join()
        self.robot_node.destroy_node()
        rclpy.try_shutdown()

    def heartbeat(self):
        while not self.heartbeat_stop_event.wait(2.0):
            print("T2KitchenAgent heartbeat", flush=True)
