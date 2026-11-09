"""Standalone T2 ROS camera viewer with YOLO detection boxes.

Run with ``python -m t2_server_side.vision.object_detection.viewer``.
The viewer discovers the active camera topic and message type. Use
``--rgb-topic`` to select a particular camera when several are published.
Press q or Escape in the RGB window to quit.
"""

import argparse
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

from .main import inference


class T2DetectionViewer(Node):
    RGB_WINDOW = "T2 RGB Detection"
    DEPTH_WINDOW = "T2 Depth"
    STUDIO_TOPIC = "/t2/vision/detection/image_compressed"

    def __init__(self, rgb_topic=None, depth_topic=None, publish_topic=None):
        super().__init__("t2_detection_viewer")
        self.latest_rgb = None
        self.latest_rgb_header = None
        self.latest_depth = None
        self.frame_number = 0
        self._reported_rgb_error = False
        self._reported_depth_error = False
        self.requested_rgb_topic = rgb_topic
        self.requested_depth_topic = depth_topic
        self.rgb_topic = None
        self.rgb_subscription = None
        self.depth_subscription = None
        self.annotated_publisher = (
            self.create_publisher(CompressedImage, publish_topic, 1)
            if publish_topic else None
        )
        if publish_topic:
            self.get_logger().info(f"Annotated RGB output: {publish_topic}")
        self._last_wait_log = time.monotonic()
        self.create_timer(1.0, self._discover_topics)
        self._discover_topics()
        self.get_logger().info("Waiting for a simulator RGB camera topic")

    def _discover_topics(self):
        available = {
            name: set(types) for name, types in self.get_topic_names_and_types()
        }
        if self.rgb_subscription is None:
            candidates = []
            for name, types in available.items():
                if self.requested_rgb_topic and name != self.requested_rgb_topic:
                    continue
                if not self.requested_rgb_topic and not (
                    name.endswith("/rgb/image_raw")
                    or name.endswith("/rgb/image_compressed")
                ):
                    continue
                if "sensor_msgs/msg/CompressedImage" in types:
                    candidates.append((name, CompressedImage, self._on_compressed_rgb))
                elif "sensor_msgs/msg/Image" in types:
                    candidates.append((name, Image, self._on_rgb))
            # Prefer the head camera and the simulator's default compressed stream.
            candidates.sort(key=lambda item: (
                "head" not in item[0].lower(),
                item[1] is not CompressedImage,
                item[0],
            ))
            if candidates:
                name, message_type, callback = candidates[0]
                self.rgb_subscription = self.create_subscription(
                    message_type, name, callback, qos_profile_sensor_data
                )
                self.rgb_topic = name
                self.get_logger().info(
                    f"RGB: {name} ({message_type.__name__})"
                )

        if self.rgb_topic and self.requested_depth_topic != "" and self.depth_subscription is None:
            depth_topic = self.requested_depth_topic or (
                self.rgb_topic.rsplit("/rgb/", 1)[0] + "/depth/image_raw"
            )
            if "sensor_msgs/msg/Image" in available.get(depth_topic, set()):
                self.depth_subscription = self.create_subscription(
                    Image, depth_topic, self._on_depth, qos_profile_sensor_data
                )
                self.get_logger().info(f"Depth: {depth_topic}")

        if self.rgb_subscription is None and time.monotonic() - self._last_wait_log >= 10:
            self.get_logger().warning(
                "No RGB topic yet; start the simulator camera or pass --rgb-topic"
            )
            self._last_wait_log = time.monotonic()

    def _on_rgb(self, message):
        try:
            encoding = message.encoding.lower()
            if encoding not in ("rgb8", "bgr8"):
                raise ValueError(f"unsupported RGB encoding: {message.encoding}")
            if message.width <= 0 or message.height <= 0:
                raise ValueError("empty RGB image")
            if message.step < message.width * 3:
                raise ValueError("invalid RGB row stride")
            data = np.frombuffer(message.data, dtype=np.uint8)
            expected = message.height * message.step
            if data.size < expected:
                raise ValueError("short RGB image buffer")
            rows = data[:expected].reshape(message.height, message.step)
            image = rows[:, :message.width * 3].reshape(
                message.height, message.width, 3
            )
            self.latest_rgb = (
                image.copy() if encoding == "rgb8"
                else cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            )
            self.latest_rgb_header = message.header
            self.frame_number += 1
        except (ValueError, cv2.error) as error:
            if not self._reported_rgb_error:
                self.get_logger().error(str(error))
                self._reported_rgb_error = True

    def _on_compressed_rgb(self, message):
        try:
            encoded = np.frombuffer(message.data, dtype=np.uint8)
            bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError("cannot decode compressed RGB image")
            self.latest_rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            self.latest_rgb_header = message.header
            self.frame_number += 1
        except (ValueError, cv2.error) as error:
            if not self._reported_rgb_error:
                self.get_logger().error(str(error))
                self._reported_rgb_error = True

    def _on_depth(self, message):
        try:
            encoding = message.encoding.upper()
            if encoding == "32FC1":
                dtype = np.dtype(">f4" if message.is_bigendian else "<f4")
                scale = 1.0
            elif encoding == "16UC1":
                dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
                scale = 0.001  # ROS 16UC1 depth is conventionally in millimeters.
            else:
                raise ValueError(f"unsupported depth encoding: {message.encoding}")
            if message.width <= 0 or message.height <= 0:
                raise ValueError("empty depth image")
            row_bytes = message.width * dtype.itemsize
            if message.step < row_bytes:
                raise ValueError("invalid depth row stride")
            data = np.frombuffer(message.data, dtype=np.uint8)
            expected = message.height * message.step
            if data.size < expected:
                raise ValueError("short depth image buffer")
            rows = data[:expected].reshape(message.height, message.step)
            depth = np.ascontiguousarray(rows[:, :row_bytes]).view(dtype).reshape(
                message.height, message.width
            )
            self.latest_depth = depth.astype(np.float32) * scale
        except ValueError as error:
            if not self._reported_depth_error:
                self.get_logger().error(str(error))
                self._reported_depth_error = True

    def publish_annotated(self, bgr):
        if self.annotated_publisher is None:
            return
        ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            raise RuntimeError("cannot encode annotated RGB image")
        message = CompressedImage()
        if self.latest_rgb_header is not None:
            message.header = self.latest_rgb_header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.annotated_publisher.publish(message)


def draw_detections(bgr, detections):
    for detection in detections:
        x, y, width, height = detection["bbox"]
        color = (0, 220, 0)
        cv2.rectangle(bgr, (x, y), (x + width, y + height), color, 2)
        text = f"{detection['label']} {detection['confidence']:.2f}"
        position = (x, max(14, y - 5))
        cv2.putText(
            bgr, text, position, cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (0, 0, 0), 3, cv2.LINE_AA,
        )
        cv2.putText(
            bgr, text, position, cv2.FONT_HERSHEY_SIMPLEX,
            0.45, color, 1, cv2.LINE_AA,
        )


def colorize_depth(depth):
    valid = np.isfinite(depth) & (depth > 0)
    clipped = np.clip(depth, 0.2, 10.0)
    normalized = np.zeros(depth.shape, dtype=np.uint8)
    normalized[valid] = ((clipped[valid] - 0.2) * (255.0 / 9.8)).astype(np.uint8)
    bgr = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    bgr[~valid] = 0
    return bgr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-topic", default=None,
                        help="Camera topic; default: discover a head RGB camera")
    parser.add_argument("--depth-topic", default=None,
                        help="Depth topic; default: matching RGB camera, or '' to disable")
    parser.add_argument("--model", default=None)
    parser.add_argument("--conf", type=float, default=None)
    parser.add_argument("--device", default=None)
    parser.add_argument("--studio-only", action="store_true",
                        help="Publish annotated RGB for Booster Studio without a separate window")
    parser.add_argument("--publish-topic", default=None,
                        help="Publish annotated JPEG on a ROS image topic")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = None
    windows_open = False
    last_frame_number = -1
    inference_failed = False
    try:
        publish_topic = args.publish_topic or (
            T2DetectionViewer.STUDIO_TOPIC if args.studio_only else None
        )
        depth_topic = "" if args.studio_only and args.depth_topic is None else args.depth_topic
        node = T2DetectionViewer(args.rgb_topic, depth_topic, publish_topic)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.latest_rgb is not None and node.frame_number != last_frame_number:
                rgb = node.latest_rgb
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if not inference_failed:
                    try:
                        detections = inference(
                            rgb, model_path=args.model,
                            confidence=args.conf, device=args.device,
                        )
                        draw_detections(bgr, detections)
                    except Exception as error:
                        inference_failed = True
                        node.get_logger().error(
                            f"YOLO disabled: {type(error).__name__}: {error}"
                        )
                node.publish_annotated(bgr)
                if not args.studio_only:
                    cv2.imshow(node.RGB_WINDOW, bgr)
                    if args.depth_topic != "" and node.latest_depth is not None:
                        cv2.imshow(node.DEPTH_WINDOW, colorize_depth(node.latest_depth))
                    windows_open = True
                last_frame_number = node.frame_number
            if windows_open:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if cv2.getWindowProperty(node.RGB_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if windows_open:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
