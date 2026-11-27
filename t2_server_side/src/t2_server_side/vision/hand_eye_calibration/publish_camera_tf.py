#!/usr/bin/env python3
"""Publish the hand-eye calibration result as a static TF: <mount> -> <camera frame>.

    python3 publish_camera_tf.py                               # results found automatically (below)
    python3 publish_camera_tf.py --result output_hand_right/handeye.json --result output_hand_left/handeye.json
    python3 publish_camera_tf.py --frame rgbd_camera_frame     # skip reading the frame name from camera_info

Results: every --result file (handeye.json written by solve.py), averaged when there are several;
default: output_hand_right + output_hand_left if present, else output_odom, else output.
All results must have the same mount link (head_pitch_link).
Camera frame: header.frame_id of the camera_info topic (default: found like collect.py), so the TF
matches the frame the images are stamped with. The transform is the optical frame (x right, y down,
z forward), which is what image pixels / depth points are expressed in.
Published on /t2/tf_static (the TF tree every server-side node uses; --tf sim for /tf_static),
latched: keep this running (it only republishes on start). Refuses if someone already publishes a
parent for that frame (two parents break the TF tree), unless --force.

Then anywhere on the server, a point seen by the camera goes to the robot with plain tf2:
    T = tf_buffer.lookup_transform("trunk", msg.header.frame_id, msg.header.stamp)   # or odom / map
"""
import argparse
import json
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from tf2_msgs.msg import TFMessage

import handeye as he

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_RESULTS = [["output_hand_right", "output_hand_left"], ["output_odom"], ["output"]]
STATIC_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_results(paths):
    if not paths:
        for group in DEFAULT_RESULTS:
            found = [os.path.join(HERE, d, "handeye.json") for d in group
                     if os.path.exists(os.path.join(HERE, d, "handeye.json"))]
            if found:
                paths = found
                break
    if not paths:
        sys.exit("no handeye.json found: run solve.py first or pass --result")
    results = [json.load(open(p)) for p in paths]
    mounts = {r["mount"] for r in results}
    if len(mounts) != 1:
        sys.exit(f"results have different mount links {mounts}")
    Ts = [np.array(r["T_mount_cam"]["matrix"]) for r in results]
    # average: mean translation, rotation = projection of the mean rotation matrix onto SO(3)
    U, _, Vt = np.linalg.svd(sum(T[:3, :3] for T in Ts) / len(Ts))
    R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    T = he.pose(R, np.mean([T[:3, 3] for T in Ts], axis=0))
    for p, Ti in zip(paths, Ts):
        E = he.inv(T) @ Ti
        print(f"  {os.path.relpath(p, HERE)}: {np.linalg.norm(E[:3, 3]) * 1000:.1f} mm, "
              f"{he.angle_deg(E[:3, :3]):.2f} deg from the average")
    return mounts.pop(), T


class CameraTf(Node):
    def __init__(self):
        super().__init__("t2_camera_tf")
        self.frame = None
        self.parents = {}                                   # child frame -> parent seen on TF

    def watch_tf(self, tf_topic, static_topic):
        cb = lambda m: self.parents.update({t.child_frame_id: t.header.frame_id for t in m.transforms})
        self.create_subscription(TFMessage, tf_topic, cb, 100)
        self.create_subscription(TFMessage, static_topic, cb, STATIC_QOS)

    def watch_camera_info(self, topic):
        self.create_subscription(CameraInfo, topic, lambda m: setattr(self, "frame", m.header.frame_id),
                                 qos_profile_sensor_data)


def spin_for(node, seconds, until=lambda: False):
    end = node.get_clock().now().nanoseconds + seconds * 1e9
    while rclpy.ok() and node.get_clock().now().nanoseconds < end and not until():
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--result", action="append", help="handeye.json (repeat to average several)")
    ap.add_argument("--frame", help="camera frame id; default: header.frame_id of camera_info")
    ap.add_argument("--info", help="camera_info topic; default: the head RGB camera's, found automatically")
    ap.add_argument("--tf", choices=["t2", "sim"], default="t2", help="t2: /t2/tf_static (default), sim: /tf_static")
    ap.add_argument("--force", action="store_true", help="publish even if the frame already has a parent")
    a = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)            # show progress even without a tty (docker exec)

    print("hand-eye results:")
    mount, T = load_results(a.result)
    tf_topic, static_topic = ("/t2/tf", "/t2/tf_static") if a.tf == "t2" else ("/tf", "/tf_static")
    rclpy.init()
    node = CameraTf()
    node.watch_tf(tf_topic, static_topic)
    frame = a.frame
    if not frame:
        info = a.info
        if not info:
            from collect import find_image_topic
            print("looking for the camera topic (up to 10 s) ...")
            spin_for(node, 10, lambda: find_image_topic(node) is not None)
            image = find_image_topic(node)
            if not image:
                sys.exit("no camera topic found: pass --frame or --info")
            info = image.rsplit("/", 1)[0] + "/camera_info"
        node.watch_camera_info(info)
        print(f"waiting for {info} ...")
        spin_for(node, 30, lambda: node.frame is not None)
        if not node.frame:
            sys.exit(f"no message on {info}: pass --frame")
        frame = node.frame
    spin_for(node, 2)                                       # collect the existing TF
    if frame in node.parents and not a.force:
        sys.exit(f"{frame} already has parent {node.parents[frame]} on {tf_topic}/{static_topic}; "
                 f"two parents break the TF tree (--force to publish anyway)")

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id, t.child_frame_id = mount, frame
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T[:3, 3])
    x, y, z, w = he.R_to_quat_xyzw(T[:3, :3])
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, (x, y, z, w))
    pub = node.create_publisher(TFMessage, static_topic, STATIC_QOS)
    pub.publish(TFMessage(transforms=[t]))
    d = he.describe(T)
    print(f"{static_topic}: {mount} -> {frame}  xyz {np.round(d['xyz'], 4).tolist()}  "
          f"rpy {np.round(d['rpy'], 4).tolist()} (latched; keep this running)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
