"""Save unannotated T2 camera frames for manual YOLO labeling.

Run: python -m t2_server_side.vision.capture
In a terminal, press p to save one frame and q/Esc to quit.
Use --preview to show the live image when a graphical display is available.
"""

import argparse
from contextlib import contextmanager
from datetime import datetime, timezone
import os
from pathlib import Path
import select
import sys
import termios
import tty
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class FrameCapture(Node):
    def __init__(self, output_dir, topic=None):
        super().__init__("t2_frame_capture")
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.requested_topic = topic
        self.topic = None
        self.subscription = None
        self.latest_bgr = None
        self.frame_number = 0
        self.saved_frame_number = -1
        self._reported_error = False
        self.create_timer(1.0, self._discover_topic)
        self._discover_topic()

    def _discover_topic(self):
        if self.subscription is not None:
            return
        candidates = []
        for name, types in self.get_topic_names_and_types():
            if self.requested_topic:
                if name != self.requested_topic:
                    continue
            elif not name.endswith(("/rgb/image_raw", "/rgb/image_compressed")):
                continue
            if "sensor_msgs/msg/Image" in types:
                candidates.append((name, Image, self._on_raw))
            elif "sensor_msgs/msg/CompressedImage" in types:
                candidates.append((name, CompressedImage, self._on_compressed))
        candidates.sort(key=lambda item: (
            "head" not in item[0].lower(),
            item[1] is not Image,
            item[0],
        ))
        if candidates:
            self.topic, message_type, callback = candidates[0]
            self.subscription = self.create_subscription(
                message_type, self.topic, callback, qos_profile_sensor_data
            )
            self.get_logger().info(f"Camera: {self.topic} ({message_type.__name__})")

    def _on_raw(self, message):
        try:
            encoding = message.encoding.lower()
            if encoding not in ("rgb8", "bgr8"):
                raise ValueError(f"unsupported RGB encoding: {message.encoding}")
            if message.width <= 0 or message.height <= 0 or message.step < message.width * 3:
                raise ValueError("invalid RGB image dimensions or row stride")
            data = np.frombuffer(message.data, dtype=np.uint8)
            size = message.height * message.step
            if data.size < size:
                raise ValueError("short RGB image buffer")
            rows = data[:size].reshape(message.height, message.step)
            image = rows[:, :message.width * 3].reshape(message.height, message.width, 3)
            self.latest_bgr = (
                cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if encoding == "rgb8" else image.copy()
            )
            self.frame_number += 1
        except (ValueError, cv2.error) as error:
            self._log_frame_error(error)

    def _on_compressed(self, message):
        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("cannot decode compressed RGB image")
            self.latest_bgr = image
            self.frame_number += 1
        except (ValueError, cv2.error) as error:
            self._log_frame_error(error)

    def _log_frame_error(self, error):
        if not self._reported_error:
            self.get_logger().error(str(error))
            self._reported_error = True

    def save_frame(self):
        if self.latest_bgr is None:
            self.get_logger().warning("No image yet; check the camera topic and ROS_DOMAIN_ID")
            return None
        if self.frame_number == self.saved_frame_number:
            self.get_logger().warning("No new frame since the last capture")
            return None
        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        path = self.output_dir / f"frame_{stamp}_{uuid4().hex[:8]}.png"
        if not cv2.imwrite(str(path), self.latest_bgr):
            raise OSError(f"Cannot write {path}")
        self.saved_frame_number = self.frame_number
        self.get_logger().info(f"Saved: {path}")
        return path


@contextmanager
def terminal_keys():
    """Read single keys without Enter and restore the terminal on exit."""
    if not sys.stdin.isatty():
        yield False
        return
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield True
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def main():
    default_dir = Path(__file__).resolve().parent / "dataset" / "captures" / "images"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default=os.environ.get("T2_CAPTURE_DIR", str(default_dir)),
                        help="Destination for unannotated PNG images")
    parser.add_argument("--topic", default=None,
                        help="RGB camera topic; default: discover head camera")
    parser.add_argument("--preview", action="store_true",
                        help="Show a live OpenCV window; terminal capture works without one")
    args, ros_args = parser.parse_known_args()
    if not sys.stdin.isatty() and not args.preview:
        parser.error("An interactive terminal is required for p/q keys; use --preview with a display")

    rclpy.init(args=ros_args)
    node = None
    window = "T2 Raw Capture - p: save / q: quit"
    window_open = False
    try:
        node = FrameCapture(args.output, args.topic)
        print(f"ROS_DOMAIN_ID: {os.environ.get('ROS_DOMAIN_ID', '0')}\n"
              f"Output: {node.output_dir}\nWaiting for RGB frames. p: save, q/Esc: quit.",
              flush=True)
        with terminal_keys() as has_terminal:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.02)
                keys = []
                if args.preview and node.latest_bgr is not None:
                    cv2.imshow(window, node.latest_bgr)
                    window_open = True
                    key = cv2.waitKey(1) & 0xFF
                    if key != 255:
                        keys.append(key)
                    if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
                if has_terminal and select.select([sys.stdin], [], [], 0)[0]:
                    keys.append(ord(sys.stdin.read(1).lower()))
                if any(key in (ord("q"), 27) for key in keys):
                    break
                if ord("p") in keys:
                    node.save_frame()
    except KeyboardInterrupt:
        pass
    finally:
        if window_open:
            cv2.destroyWindow(window)
            cv2.waitKey(1)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
