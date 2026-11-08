import threading

import rclpy
from booster_agent_framework import AgentBase, AgentFeatures
from rclpy.executors import SingleThreadedExecutor

from .sensors.e1r import E1RMount
from .ros.nav2_node import T2Nav2Node
from .ros.node import T2RobotNode
from .robot.relay_bridges.node import T2RelayNode
from .robot.t2 import T2Robot


class T2KitchenAgent(AgentBase):
    def __init__(self):
        super().__init__(AgentFeatures())

        rclpy.init()

        self.t2 = T2Robot()
        self.robot_node = T2RobotNode(self.t2)
        self.nav2_node = T2Nav2Node(self.t2)
        self.relay_node = T2RelayNode()

        # E1R 3d-lidar
        self.e1r_mount = E1RMount(self.relay_node)

        self.robot_executor = SingleThreadedExecutor()
        self.robot_executor.add_node(self.robot_node)
        self.robot_executor_thread = threading.Thread(
            target=self.robot_executor.spin,
            daemon=True,
        )

        self.nav2_executor = SingleThreadedExecutor()
        self.nav2_executor.add_node(self.nav2_node)
        self.nav2_executor_thread = threading.Thread(
            target=self.nav2_executor.spin,
            daemon=True,
        )

        self.relay_executor = SingleThreadedExecutor()
        self.relay_executor.add_node(self.relay_node)
        self.relay_executor_thread = threading.Thread(
            target=self.relay_executor.spin,
            daemon=True,
        )
        self.executors_started = False

    def on_agent_activated(self):
        if not self.executors_started:
            self.t2.launch(self.relay_node)
            self.relay_executor_thread.start()
            self.nav2_executor_thread.start()
            self.robot_executor_thread.start()
            self.executors_started = True

    def on_agent_close(self):
        self.t2.stop_nav2()
        self.robot_executor.shutdown()
        self.nav2_executor.shutdown()
        self.relay_executor.shutdown()
        if self.executors_started:
            self.robot_executor_thread.join()
            self.nav2_executor_thread.join()
            self.relay_executor_thread.join()
        self.robot_node.destroy_node()
        self.nav2_node.destroy_node()
        self.relay_node.destroy_node()
        rclpy.try_shutdown()
