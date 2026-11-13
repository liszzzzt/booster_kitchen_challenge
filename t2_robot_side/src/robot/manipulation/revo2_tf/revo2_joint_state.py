#!/usr/bin/env python3
"""Revo2 finger joint states for TF:  /revo2/hand_state (closure 0..1)  ->  sensor_msgs/JointState.

Booster's robot_state_publisher only knows the 31 T2 joints, so the finger frames are missing from
/tf. This node turns the hand state into joint angles of the hand URDFs (revo2_description/
revo2_{left,right}.urdf) and publishes them; two extra robot_state_publisher instances (see
revo2_tf.launch.py) then put the finger frames under <side>_wrist_roll_link in /tf and /tf_static.

    in   /revo2/hand_state            std_msgs/String JSON from the revo2_hand sim plugin, 20 Hz
                                      {"robot", "side", "closure": {dof: 0..1}, "target": {...}, "time"}
         /joint_states                (optional) only for the time stamp, so finger TF and arm TF
                                      carry the same stamps (lookups at one time then work)
    out  /revo2/<side>/joint_states   JointState, the 11 joints of one hand (thumb 3, fingers 2 each)
         /revo2/joint_states          JointState, both hands

closure -> angle:  proximal/metacarpal = lower + closure * (upper - lower)  (same as the plugin's PD target),
distal = multiplier * proximal + offset  (URDF <mimic>: thumb x1.0, fingers x1.155). Limits and mimic
are read from the URDFs, so they are the only source of these numbers.

    python3 revo2_joint_state.py [--ros-args -p robot:=robot1 -p rate_hz:=50.0 ...]

Parameters: robot (robot1; "" = any), hand_state_topic, sync_topic (/joint_states; "" = use own clock),
rate_hz (50, max publish rate), use_target (false: measured closure; true: commanded closure),
urdf_dir (default: revo2_description next to this file).
For another hand-state source (e.g. the real Revo2 SDK) feed Revo2HandModel.positions() with closures.
"""
from __future__ import annotations

import json
import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
SIDES = ("left", "right")
# hand_state dof -> driven URDF joint (without the "<side>_" prefix)
DOF_JOINT = {
    "thumb_rotation": "thumb_metacarpal_joint",
    "thumb_flex": "thumb_proximal_joint",
    "index": "index_proximal_joint",
    "middle": "middle_proximal_joint",
    "ring": "ring_proximal_joint",
    "pinky": "pinky_proximal_joint",
}


class Revo2HandModel:
    """Joint names, limits and mimic rules of one hand, read from its URDF. No ROS needed."""

    def __init__(self, urdf_path: str, side: str):
        self.side = side
        root = ET.parse(urdf_path).getroot()
        self.names: List[str] = []                               # all revolute joints, URDF order
        self.limits: Dict[str, Tuple[float, float]] = {}
        self.mimic: Dict[str, Tuple[str, float, float]] = {}     # joint -> (master, multiplier, offset)
        for j in root.findall("joint"):
            if j.get("type") != "revolute":
                continue
            n = j.get("name")
            lim = j.find("limit")
            self.names.append(n)
            self.limits[n] = (float(lim.get("lower")), float(lim.get("upper")))
            m = j.find("mimic")
            if m is not None:
                self.mimic[n] = (m.get("joint"), float(m.get("multiplier", 1.0)), float(m.get("offset", 0.0)))
        self.driven = {dof: f"{side}_{j}" for dof, j in DOF_JOINT.items()}
        missing = [j for j in self.driven.values() if j not in self.limits]
        if missing:
            raise ValueError(f"{urdf_path}: joints {missing} not found")

    def positions(self, closure: Dict[str, float]) -> Dict[str, float]:
        """closure {dof: 0..1} (missing dofs = 0) -> {joint: angle [rad]} for all revolute joints."""
        q = {n: 0.0 for n in self.names}
        for dof, joint in self.driven.items():
            c = min(1.0, max(0.0, float(closure.get(dof, 0.0))))
            lo, hi = self.limits[joint]
            q[joint] = lo + c * (hi - lo)
        for joint, (master, mul, off) in self.mimic.items():
            lo, hi = self.limits[joint]
            q[joint] = min(hi, max(lo, mul * q[master] + off))
        return q


def parse_hand_state(text: str, robot: str, use_target: bool) -> Optional[Tuple[str, Dict[str, float]]]:
    """One /revo2/hand_state message -> (side, closure) or None if it is not for us / malformed."""
    try:
        d = json.loads(text)
    except ValueError:
        return None
    if robot and d.get("robot", robot) != robot:
        return None
    side = d.get("side")
    closure = d.get("target" if use_target else "closure")
    if side not in SIDES or not isinstance(closure, dict):
        return None
    return side, closure


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    class Revo2JointStatePublisher(Node):
        def __init__(self):
            super().__init__("revo2_joint_state_publisher")
            self.robot = self.declare_parameter("robot", "robot1").value
            hs_topic = self.declare_parameter("hand_state_topic", "/revo2/hand_state").value
            sync_topic = self.declare_parameter("sync_topic", "/joint_states").value
            rate = float(self.declare_parameter("rate_hz", 50.0).value)
            self.use_target = bool(self.declare_parameter("use_target", False).value)
            urdf_dir = self.declare_parameter("urdf_dir", os.path.join(HERE, "revo2_description")).value

            self.models = {s: Revo2HandModel(os.path.join(urdf_dir, f"revo2_{s}.urdf"), s) for s in SIDES}
            self.q = {s: self.models[s].positions({}) for s in SIDES}     # open hands until data arrives
            self.got_state = False
            self.min_dt = 1.0 / rate if rate > 0 else 0.0
            self.last_pub = None
            self.last_sync = None

            self.pub = {s: self.create_publisher(JointState, f"/revo2/{s}/joint_states", 10) for s in SIDES}
            self.pub_all = self.create_publisher(JointState, "/revo2/joint_states", 10)
            self.create_subscription(String, hs_topic, self.on_hand_state, 10)
            if sync_topic:      # best effort: matches both reliable (Studio) and sensor-data (/t2/...) publishers
                self.create_subscription(JointState, sync_topic, self.on_sync, qos_profile_sensor_data)
            # fallback when the sync topic is silent: publish on our own clock
            self.create_timer(max(self.min_dt, 0.02), self.on_timer)
            self.get_logger().info(
                f"{hs_topic} -> /revo2/{{left,right}}/joint_states + /revo2/joint_states "
                f"(stamp from {sync_topic or 'own clock'}, robot {self.robot or 'any'}, "
                f"{'target' if self.use_target else 'measured'} closure)")

        def on_hand_state(self, msg):
            r = parse_hand_state(msg.data, self.robot, self.use_target)
            if r is None:
                return
            side, closure = r
            self.q[side] = self.models[side].positions(closure)
            if not self.got_state:
                self.got_state = True
                self.get_logger().info("receiving hand state")

        def on_sync(self, msg):
            now = self.get_clock().now()
            self.last_sync = now
            stamp = msg.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = now.to_msg()
            self.publish(stamp, now)

        def on_timer(self):
            now = self.get_clock().now()
            if self.last_sync is not None and (now - self.last_sync).nanoseconds < 0.5e9:
                return                                    # sync topic is alive, it drives publishing
            self.publish(now.to_msg(), now)

        def publish(self, stamp, now):
            if self.last_pub is not None and (now - self.last_pub).nanoseconds < self.min_dt * 1e9 * 0.9:
                return
            self.last_pub = now
            all_msg = JointState()
            all_msg.header.stamp = stamp
            for s in SIDES:
                m = JointState()
                m.header.stamp = stamp
                m.name = list(self.models[s].names)
                m.position = [self.q[s][n] for n in m.name]
                self.pub[s].publish(m)
                all_msg.name += m.name
                all_msg.position += m.position
            self.pub_all.publish(all_msg)

    rclpy.init()
    node = Revo2JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
