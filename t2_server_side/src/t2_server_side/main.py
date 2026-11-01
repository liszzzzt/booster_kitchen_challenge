import rclpy

from t2_server_side.ros.node import T2ServerNode


def main():
    rclpy.init()
    node = T2ServerNode()

    try:
        rclpy.spin_once(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()