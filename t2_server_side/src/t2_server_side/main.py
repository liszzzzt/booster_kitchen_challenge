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
    schedule_thread = None

    def start_schedule():
        nonlocal schedule_thread

        if schedule_thread is not None and schedule_thread.is_alive():
            print("Schedule is already running.")
            return

        if not server_node.check_door():
            print("Door is closed. Schedule stopped.")
            return

        def run_schedule():
            print("Door is open. Schedule started.")
            center_control.execute_schedule()
            print("Schedule finished. Remote control enabled.")

        schedule_thread = threading.Thread(target=run_schedule, daemon=True)
        schedule_thread.start()

    remote_control = RemoteControl(server_node, start_schedule)

    try:
        if not server_node.wait_for_robot():
            print("Robot side is not available.")
            return

        mode = input("1: remote control, 2: schedule: ")

        if mode == "2":
            start_schedule()
        elif mode != "1":
            print("Unknown mode.")
            return

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
