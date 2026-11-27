"""/joint_states -> /t2/joint_states, with the 22 Revo2 finger joints appended once a hand reports.

Booster's /joint_states has only the 31 T2 joints. The hand state (closure 0..1 per motor) comes from
/revo2/hand_state (Booster Studio, revo2_hand plugin) or /t2/revo2/hand_state (real robot, Revo2Relay)
and is turned into joint angles with the limits and <mimic> rules of the hand URDFs
(manipulation/revo2_tf/revo2_description/revo2_<side>.urdf, same for both sides, all lower limits 0):
    driven joint = closure * upper limit,   distal joint = multiplier * proximal (clamped to its limit)
Users: MoveIt (needs the finger joints in /t2/joint_states) and revo2_tf.launch.py (finger TF).
Booster's own /joint_states is not touched.
"""
import json

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# hand_state dof -> (driven joint without "<side>_" / "_joint", upper limit [rad])
DRIVEN = {
    "thumb_rotation": ("thumb_metacarpal", 1.57),
    "thumb_flex": ("thumb_proximal", 1.03),
    "index": ("index_proximal", 1.41),
    "middle": ("middle_proximal", 1.41),
    "ring": ("ring_proximal", 1.41),
    "pinky": ("pinky_proximal", 1.41),
}
# coupled joint -> (master joint, multiplier, upper limit [rad])
MIMIC = {
    "thumb_distal": ("thumb_proximal", 1.0, 1.03),
    "index_distal": ("index_proximal", 1.155, 1.63),
    "middle_distal": ("middle_proximal", 1.155, 1.63),
    "ring_distal": ("ring_proximal", 1.155, 1.63),
    "pinky_distal": ("pinky_proximal", 1.155, 1.63),
}
HAND_STATE_TOPICS = ("/revo2/hand_state", "/t2/revo2/hand_state")


def finger_positions(side, closure):
    """closure {dof: 0..1} (missing = 0) -> {"<side>_<joint>_joint": angle} for the 11 joints of a hand."""
    q = {}
    for dof, (joint, upper) in DRIVEN.items():
        q[joint] = upper * min(1.0, max(0.0, float(closure.get(dof, 0.0))))
    for joint, (master, multiplier, upper) in MIMIC.items():
        q[joint] = min(upper, max(0.0, multiplier * q[master]))
    return {f"{side}_{joint}_joint": angle for joint, angle in q.items()}


class JointStatesRelay:
    def __init__(self, node):
        self.fingers = {}                                       # side -> {joint: angle}
        self.publisher = node.create_publisher(
            JointState,
            "/t2/joint_states",
            qos_profile_sensor_data,
        )
        self.subscription = node.create_subscription(
            JointState,
            "/joint_states",
            self._relay,
            qos_profile_sensor_data,
        )
        self.hand_subscriptions = [node.create_subscription(String, topic, self._on_hand_state, 10)
                                   for topic in HAND_STATE_TOPICS]

    def _on_hand_state(self, msg):
        try:
            state = json.loads(msg.data)
            side, closure = state["side"], state["closure"]
        except (ValueError, KeyError, TypeError):
            return
        if side in ("left", "right") and isinstance(closure, dict):
            self.fingers[side] = finger_positions(side, closure)

    def _relay(self, msg):
        if self.fingers:
            present = set(msg.name)
            extra = {n: v for hand in self.fingers.values() for n, v in hand.items() if n not in present}
            count = len(msg.name)
            msg.name = list(msg.name) + list(extra)
            msg.position = list(msg.position) + list(extra.values())
            if len(msg.velocity) == count:                       # keep the optional arrays consistent
                msg.velocity = list(msg.velocity) + [0.0] * len(extra)
            if len(msg.effort) == count:
                msg.effort = list(msg.effort) + [0.0] * len(extra)
        self.publisher.publish(msg)
