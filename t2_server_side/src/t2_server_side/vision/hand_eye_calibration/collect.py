#!/usr/bin/env python3
"""Collect hand-eye samples: checkerboard corners in the head camera + T_mount_ref from TF.

    python3 collect.py                    # Booster Studio: /tf, camera topic found automatically
    python3 collect.py --tf t2 --image <topic> --info <camera_info topic>     # real robot via /t2/tf

Static board (default): put the board 0.6-1 m in front of the robot, robot standing still, and move the
head between captures (other terminal: remote control r/t = yaw, y/u = pitch). Use --ref odom: the
robot drifts slowly in walk mode, odom follows it, trunk does not.
Board on the back of a hand instead (Studio: robot_t2_calib.basset from simulation/hand/tools/add_calib_board.py):
--board hand --ref right_wrist_roll_link, keep the head still and move the arm.

Terminal (default): Enter capture | u + Enter undo last | q + Enter quit. The camera view with the
detected corners is written to <out>/preview.jpg twice a second (open it on Windows, it refreshes), and
the terminal says when the board appears / disappears.
--gui: OpenCV window instead (needs an X server for T2WS), keys SPACE / u / q.
A capture takes 5 frames and is rejected if the board or the head moved between them.
Samples are appended to <out>/samples.npz (images in <out>/images/); delete <out> to start over.
Aim for 15+ samples with the board in different parts of the image and the head turned both ways.
"""
import argparse
import os
import queue
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

import board
import handeye as he

HERE = os.path.dirname(os.path.abspath(__file__))
TF_TOPICS = {"sim": ("/tf", "/tf_static"), "t2": ("/t2/tf", "/t2/tf_static")}
STILL_MM, STILL_DEG, FRAMES = 2.0, 0.3, 5


def find_image_topic(node):
    """A head RGB topic: prefer raw over compressed, 'rgbd_camera'/'head' over others."""
    topics = dict(node.get_topic_names_and_types())
    candidates = [(name, types[0]) for name, types in topics.items()
                  if name.endswith(("/rgb/image_raw", "/rgb/image_compressed"))]
    candidates.sort(key=lambda c: ("rgbd_camera" not in c[0] and "head" not in c[0], c[0].endswith("compressed")))
    return candidates[0][0] if candidates else None


class Collector(Node):
    def __init__(self, a):
        super().__init__("t2_handeye_collect")
        self.a = a
        self.image = None
        self.info = None
        self.frames = queue.Queue()
        self.tf = Buffer(cache_time=Duration(seconds=10))
        tf_topic, tf_static_topic = TF_TOPICS[a.tf]
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, tf_topic, lambda m: self._on_tf(m, False), 100)
        self.create_subscription(TFMessage, tf_static_topic, lambda m: self._on_tf(m, True), static_qos)

    def start_camera(self, image_topic, info_topic):
        msg_type = CompressedImage if image_topic.endswith("compressed") else Image
        self.create_subscription(msg_type, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.get_logger().info(f"image {image_topic}, camera_info {info_topic}, tf {TF_TOPICS[self.a.tf][0]}")

    def _on_tf(self, msg, static):
        for t in msg.transforms:
            if static:
                self.tf.set_transform_static(t, "collect")
            else:
                self.tf.set_transform(t, "collect")

    def _on_info(self, msg):
        self.info = msg

    def _on_image(self, msg):
        if isinstance(msg, CompressedImage):
            img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        else:
            channels = msg.step // msg.width                       # rgb8 / bgr8 / mono8
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
            img = img.reshape(msg.height, msg.width, channels)
            if channels == 1:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif msg.encoding.lower().startswith("rgb"):
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        self.image = img
        self.frames.put(img)

    def mount_T_ref(self):
        t = self.tf.lookup_transform(self.a.mount, self.a.ref, Time()).transform   # ref -> mount
        q, p = t.rotation, t.translation
        R = he.quat_wxyz_to_R((q.w, q.x, q.y, q.z))
        return he.pose(R, (p.x, p.y, p.z))

    def camera(self):
        K = np.array(self.info.k, float).reshape(3, 3)
        D = np.array(self.info.d if len(self.info.d) else np.zeros(5), float)
        return K, D


def spin_until(node, done, timeout=10.0):
    end = node.get_clock().now() + Duration(seconds=timeout)
    while not done() and node.get_clock().now() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return done()


def capture(node, object_points, b):
    """FRAMES consecutive frames, all with the board found and nothing moving -> (corners, G, image)."""
    while not node.frames.empty():
        node.frames.get_nowait()
    K, D = node.camera()
    poses, transforms, last = [], [], None
    while len(poses) < FRAMES:
        if not spin_until(node, lambda: not node.frames.empty(), 3.0):
            return None, "no camera frames"
        img = node.frames.get_nowait()
        corners = b.detect(img)
        if corners is None:
            return None, "board not found in every frame"
        T_cam_board, pnp_rms = he.pnp(corners, object_points, K, D)
        problem = he.view_problem(T_cam_board, pnp_rms)
        if problem:
            return None, problem
        poses.append(T_cam_board)
        transforms.append(node.mount_T_ref())
        last = (corners, transforms[-1], img)
    for name, Ts, mm, deg in (("board", poses, STILL_MM, STILL_DEG), ("head", transforms, STILL_MM / 2, STILL_DEG)):
        dt = max(np.linalg.norm(T[:3, 3] - Ts[0][:3, 3]) for T in Ts) * 1000
        dr = max(he.angle_deg(Ts[0][:3, :3].T @ T[:3, :3]) for T in Ts)
        if dt > mm or dr > deg:
            return None, f"{name} still moving ({dt:.1f} mm, {dr:.2f} deg): wait and retry"
    return last, ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tf", choices=sorted(TF_TOPICS), default="sim", help="sim: /tf (Studio), t2: /t2/tf (robot relay)")
    ap.add_argument("--image", help="RGB topic (Image or CompressedImage); default: found automatically")
    ap.add_argument("--info", help="CameraInfo topic; default: <image dir>/camera_info")
    ap.add_argument("--mount", default="head_pitch_link", help="link the camera is fixed to")
    ap.add_argument("--ref", default="trunk", help="link the board is fixed relative to")
    ap.add_argument("--board", choices=sorted(board.BOARDS), default="stand", help="which board (board.py)")
    ap.add_argument("--square", type=float, help="square size [m]; default the board's (printed board: measure it)")
    ap.add_argument("--out", default=os.path.join(HERE, "output"))
    ap.add_argument("--gui", action="store_true", help="OpenCV window (needs an X server)")
    a = ap.parse_args()
    b = board.BOARDS[a.board]
    a.square = a.square or b.SQUARE

    rclpy.init()
    node = Collector(a)
    image_topic = a.image
    if not image_topic:
        spin_until(node, lambda: find_image_topic(node) is not None, 5.0)
        image_topic = find_image_topic(node)
        if not image_topic:
            sys.exit("no */rgb/image_raw or */rgb/image_compressed topic: pass --image")
    node.start_camera(image_topic, a.info or image_topic.rsplit("/", 1)[0] + "/camera_info")
    if not spin_until(node, lambda: node.info is not None and node.image is not None):
        sys.exit("no image or camera_info yet: check the topics")
    try:
        spin_until(node, lambda: node.tf.can_transform(a.mount, a.ref, Time()), 5.0)
        node.mount_T_ref()
    except Exception as error:
        sys.exit(f"no TF {a.ref} -> {a.mount} on {TF_TOPICS[a.tf][0]}: {error}\nframes seen:\n"
                 f"{node.tf.all_frames_as_string()}")

    os.makedirs(os.path.join(a.out, "images"), exist_ok=True)
    path = os.path.join(a.out, "samples.npz")
    G, corners = [], []
    if os.path.exists(path):
        old = np.load(path)
        settings = {k: str(old[k]) if k in old.files else "stand" for k in ("tf", "mount", "ref", "board")}
        if settings != {"tf": a.tf, "mount": a.mount, "ref": a.ref, "board": a.board}:
            sys.exit(f"{path} was collected with {settings}; use another --out or delete it")
        G, corners = list(old["G"]), list(old["corners"])
        print(f"appending to {path} ({len(G)} samples)")
    object_points = b.object_points(a.square)

    def save():
        K, D = node.camera()
        np.savez(path, G=np.array(G), corners=np.array(corners), K=K, D=D, square=a.square, board=a.board,
                 size=(node.info.width, node.info.height), tf=a.tf, mount=a.mount, ref=a.ref, image=image_topic)

    keys = queue.Queue()
    preview_path, preview_at, board_seen = os.path.join(a.out, "preview.jpg"), 0.0, None
    if not a.gui:
        def read_stdin():
            for line in sys.stdin:
                keys.put(line.strip()[:1] or " ")
        threading.Thread(target=read_stdin, daemon=True).start()
        print(f"Enter = capture, u = undo, q = quit   (camera view: {preview_path})")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.03)
        key = None
        if not a.gui and node.image is not None and time.monotonic() - preview_at > 0.5:
            preview_at = time.monotonic()
            view = node.image.copy()
            found = b.detect(view, fast=True)
            if found is not None:
                cv2.drawChessboardCorners(view, b.PATTERN, found.reshape(-1, 1, 2), True)
            cv2.imwrite(preview_path, view)
            if (found is not None) != board_seen:
                board_seen = found is not None
                print("board: found" if board_seen else "board: NOT found", flush=True)
        if a.gui and node.image is not None:
            view = node.image.copy()
            found = b.detect(view, fast=True)
            if found is not None:
                cv2.drawChessboardCorners(view, b.PATTERN, found.reshape(-1, 1, 2), True)
            cv2.putText(view, f"{len(G)} samples  SPACE capture  u undo  q quit", (5, 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.imshow("handeye collect", cv2.resize(view, None, fx=2, fy=2) if view.shape[1] < 640 else view)
            k = cv2.waitKey(1) & 0xFF
            key = {32: " ", ord("u"): "u", ord("q"): "q", 27: "q"}.get(k)
        if not a.gui and not keys.empty():
            key = keys.get()
        if key == "q":
            break
        if key == "u" and G:
            G.pop()
            corners.pop()
            save()
            print(f"undone, {len(G)} samples")
        if key == " ":
            sample, why = capture(node, object_points, b)
            if sample is None:
                print("rejected:", why)
                continue
            uv, g, img = sample
            G.append(g)
            corners.append(uv)
            cv2.imwrite(os.path.join(a.out, "images", f"{len(G) - 1:03d}.png"), img)
            save()
            K, D = node.camera()
            rms = he.pnp(uv, object_points, K, D)[1]
            print(f"sample {len(G)}: pnp rms {rms:.2f} px, head motion spread {he.motion_spread_deg(G):.1f} deg")
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()
    print(f"{len(G)} samples in {path}; next: python3 solve.py")


if __name__ == "__main__":
    main()
