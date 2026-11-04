import time
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor

from t2_server_side.ros.node import T2ServerNode
from t2_server_side.high_level_planner.center_control import CenterControl
from t2_server_side.remote_control.remote_control import (
    RemoteControl,
    read_key,
)


def main():
    rclpy.init()
    server_node = T2ServerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(server_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    center_control = CenterControl(server_node)
    remote_control = RemoteControl(server_node)

    time.sleep(2)
    print("Server is ready.")

    try:
        center_control.execute_schedule()
        print("Schedule finished. Remote control enabled.")
        remote_control.launch()
    finally:
        executor.shutdown()
        executor_thread.join()
        server_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
