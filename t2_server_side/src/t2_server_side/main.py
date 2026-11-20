import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor

from t2_server_side.filters.tf_jump_filter import TfJumpFilterNode
from t2_server_side.ros.nav2_client import T2Nav2Client
from t2_server_side.ros.node import T2ServerNode
from t2_server_side.high_level_planner.center_control import CenterControl
from t2_server_side.remote_control.remote_control import RemoteControl


def main():
    rclpy.init()
    server_node = T2ServerNode()
    nav2_client = T2Nav2Client()
    executor = SingleThreadedExecutor()
    executor.add_node(server_node)
    executor.add_node(nav2_client)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    tf_filter_node = TfJumpFilterNode()
    tf_filter_executor = SingleThreadedExecutor()
    tf_filter_executor.add_node(tf_filter_node)
    tf_filter_thread = threading.Thread(
        target=tf_filter_executor.spin,
        daemon=True,
    )
    tf_filter_thread.start()

    center_control = CenterControl(server_node, nav2_client)
    remote_control = RemoteControl(server_node)

    try:
        if not server_node.wait_for_robot():
            print("Robot side is not available.")
            return

        mode = input("1: schedule, 2: remote control: ")

        if mode == "2":
            remote_control.launch()
            return

        if mode != "1":
            print("Unknown mode.")
            return

        if not server_node.check_door():
            print("Door is closed. Schedule stopped. Remote control enabled.")
            remote_control.launch()
            return

        print("Door is open. Server is ready.")
        center_control.execute_schedule()
        print("Schedule finished. Remote control enabled.")
        remote_control.launch()
    finally:
        tf_filter_executor.shutdown()
        tf_filter_thread.join()
        tf_filter_node.destroy_node()
        executor.shutdown()
        executor_thread.join()
        server_node.destroy_node()
        nav2_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
