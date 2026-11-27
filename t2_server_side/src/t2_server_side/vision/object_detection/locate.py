"""3D position of a detected object in the robot frame: YOLO box + depth + camera TF.

    python3 -m t2_server_side.vision.object_detection.locate cup             # print where the cup is
    python3 -m t2_server_side.vision.object_detection.locate cup --radius 0.035 --frame odom

Pipeline (per call of ObjectLocator.locate):
  1. latest RGB frame -> inference() (main.py) -> the most confident box with that label
  2. depth image (same camera, <cam>/depth/image_raw) -> median depth of the inner 40 % of the box
     = distance to the object's front surface
  3. box centre pixel + depth + camera_info K -> point in the camera optical frame
  4. TF (/t2/tf, /t2/tf_static; the camera frame comes from hand_eye_calibration/publish_camera_tf.py)
     -> point in the target frame (default trunk)
  5. cylinder: centre = surface point + radius further along the horizontal viewing direction
Needs: the camera depth stream, publish_camera_tf.py running, YOLO weights (main.py).
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from .main import inference
from .viewer import T2DetectionViewer

STATIC_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LocateError(RuntimeError):
    pass


def _quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class ObjectLocator(T2DetectionViewer):
    def __init__(self, rgb_topic=None, depth_topic=None, tf_prefix="/t2"):
        super().__init__(rgb_topic, depth_topic, None)
        self.info = None
        self.info_subscription = None
        self.tf = Buffer(cache_time=Duration(seconds=10))
        cb = lambda m, static: [self.tf.set_transform_static(t, "locate") if static else
                                self.tf.set_transform(t, "locate") for t in m.transforms]
        self.create_subscription(TFMessage, f"{tf_prefix}/tf", lambda m: cb(m, False), 100)
        self.create_subscription(TFMessage, f"{tf_prefix}/tf_static", lambda m: cb(m, True), STATIC_QOS)
        self.create_timer(1.0, self._subscribe_info)

    def _on_depth(self, message):
        # Booster Studio publishes depth as "mono16" = uint16 millimetres (depth_publisher_extension:
        # depth_mm); the viewer only knows the ROS names 16UC1 / 32FC1. Same data as 16UC1.
        if message.encoding.lower() == "mono16":
            message.encoding = "16UC1"
        super()._on_depth(message)

    def _subscribe_info(self):
        if self.info_subscription is None and self.rgb_topic:
            topic = self.rgb_topic.rsplit("/", 1)[0] + "/camera_info"
            self.info_subscription = self.create_subscription(
                CameraInfo, topic, lambda m: setattr(self, "info", m), qos_profile_sensor_data)

    def ready(self):
        """'' when everything needed has arrived, else what is missing."""
        if self.latest_rgb is None:
            return "no RGB image yet"
        if self.depth_subscription is None:
            return f"no depth topic ({self.rgb_topic.rsplit('/rgb/', 1)[0]}/depth/image_raw)"
        if self.latest_depth is None:
            return "no depth image yet"
        if self.info is None:
            return "no camera_info yet"
        return ""

    def locate(self, label, radius=0.0, frame="trunk", confidence=None, save=None):
        """(point [x, y, z] in `frame`, details dict). Raises LocateError."""
        problem = self.ready()
        if problem:
            raise LocateError(problem)
        rgb, header = self.latest_rgb, self.latest_rgb_header
        detections = inference(rgb, confidence=confidence)
        if save:                                                # what the detector saw, for checking
            import cv2
            from .viewer import draw_detections
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            draw_detections(bgr, detections)
            cv2.imwrite(save, bgr)
        matches = sorted((d for d in detections if d["label"] == label), key=lambda d: -d["confidence"])
        if not matches:
            raise LocateError(f"no '{label}' detected (seen: {sorted({d['label'] for d in detections})})")
        det = matches[0]
        left, top, w, h = det["bbox"]
        u, v = left + w / 2.0, top + h / 2.0

        depth = self.latest_depth
        sy, sx = depth.shape[0] / rgb.shape[0], depth.shape[1] / rgb.shape[1]   # depth may be another size
        x0, x1 = int((left + 0.3 * w) * sx), int((left + 0.7 * w) * sx) + 1
        y0, y1 = int((top + 0.3 * h) * sy), int((top + 0.7 * h) * sy) + 1
        patch = depth[y0:y1, x0:x1]
        patch = patch[np.isfinite(patch) & (patch > 0.05)]
        if patch.size == 0:
            raise LocateError("no valid depth inside the box")
        z = float(np.median(patch))

        K = np.array(self.info.k, float).reshape(3, 3)
        p_cam = z * np.linalg.solve(K, [u, v, 1.0])            # optical frame: x right, y down, z forward
        cam_frame = self.info.header.frame_id or header.frame_id
        try:
            t = self.tf.lookup_transform(frame, cam_frame, Time()).transform
        except Exception as error:
            raise LocateError(f"no TF {cam_frame} -> {frame}: {error} "
                              f"(is publish_camera_tf.py running?)")
        R = _quat_to_R(t.rotation)
        origin = np.array([t.translation.x, t.translation.y, t.translation.z])
        p = origin + R @ p_cam
        if radius > 0:                                          # front surface -> cylinder axis
            ray = p - origin
            ray[2] = 0.0
            p = p + radius * ray / np.linalg.norm(ray)
        return p, {"label": label, "confidence": det["confidence"], "bbox": det["bbox"], "depth": z,
                   "camera_frame": cam_frame, "point_camera": p_cam.tolist()}


def wait_ready(node, timeout=15.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not node.ready():
            return ""
    return node.ready()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("label", help="YOLO label, e.g. cup")
    ap.add_argument("--radius", type=float, default=0.0, help="cylinder radius [m]: report the axis, not the surface")
    ap.add_argument("--frame", default="trunk")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--conf", type=float, default=None, help="detection threshold (default 0.25 / T2_YOLO_CONF)")
    ap.add_argument("--save", help="write the image with the detections drawn, e.g. /ws/locate.jpg")
    a = ap.parse_args()
    rclpy.init()
    node = ObjectLocator()
    try:
        problem = wait_ready(node)
        if problem:
            raise SystemExit(f"not ready: {problem}")
        for _ in range(a.repeat):
            for _ in range(5):                                  # fresh frames + TF
                rclpy.spin_once(node, timeout_sec=0.05)
            try:
                p, d = node.locate(a.label, a.radius, a.frame, a.conf, a.save)
                print(f"{a.label} in {a.frame}: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m  "
                      f"(conf {d['confidence']:.2f}, depth {d['depth']:.3f} m, box {d['bbox']})")
            except LocateError as error:
                print("failed:", error)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
