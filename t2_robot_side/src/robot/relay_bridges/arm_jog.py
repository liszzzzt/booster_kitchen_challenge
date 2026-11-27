"""Joint jogging of the arms (and head) from the server, e.g. the keyboard tool arm_teleop.py.

  in   /t2/arm/jog    std_msgs/String JSON, one of
         {"cmd": "jog", "side": "right"|"left"|"head", "joint": "elbow_pitch", "delta": 0.05}   [rad]
         {"cmd": "pose", "side": "right"|"left", "name": "calib"}    preset arm pose + head pitch
         {"cmd": "home"}     back to the pose at "start"
         {"cmd": "start", "legs": "lock"|"walk"}   take the arms, hold the current pose (any motion starts
                             it too, with legs "lock"). lock: legs held stiff in the standing posture
                             (custom mode, no sway / stepping); walk: walking controller keeps balance
                             (it may step / drift). See manipulation/arm_controller.py.
         {"cmd": "stop"}     release: back to Booster's control (prepare = standing, or walking)
  out  /t2/arm/state  std_msgs/String JSON {"enabled", "msg", "q": {joint: commanded angle}} after each command

Runs robot/manipulation/arm_controller.ArmController (min-jerk joint moves through upper_body_control),
so it works the same in Booster Studio and on the real robot. Targets are clamped to the joint limits
of arm_ik.py (head: yaw +-1.0, pitch -0.3..0.4). Moves run in a worker thread; jog commands that pile
up while a move runs are merged into one move.
"""
import json
import queue
import threading

from std_msgs.msg import String

# arm joints in arm_ik order + head pitch. "calib": the back of the hand (calibration board of
# simulation/hand/tools/add_calib_board.py) faces the head camera ~0.3 m away; found by IK in the
# Studio model, both hands checked: board detected, 8-12 deg off the camera axis, no self-collision.
PRESETS = {
    "calib": {
        "right": ([-1.128, 1.061, 0.765, 1.008, 1.348, -0.098, -0.685], 0.4),
        "left": ([-1.293, -1.295, 0.863, -0.419, 1.201, 0.126, 0.924], 0.4),
    },
}
HEAD = {"yaw": ("aa_head_yaw_joint", -1.0, 1.0), "pitch": ("head_pitch_joint", -0.3, 0.4)}


class ArmJogRelay:
    def __init__(self, node, client):
        self.node = node
        self.client = client
        self.arm = None
        self.home = None
        self.legs = "lock"
        self.publisher = node.create_publisher(String, "/t2/arm/state", 10)
        self.subscription = node.create_subscription(String, "/t2/arm/jog", self._on_command, 10)
        self.commands = queue.Queue()
        threading.Thread(target=self._serve, daemon=True).start()
        node.get_logger().info("arm jog relay: on")

    def _on_command(self, msg):
        self.commands.put(msg.data)

    def _serve(self):
        while True:
            text = self.commands.get()
            try:
                command = json.loads(text)
                if command.get("cmd") == "jog":            # merge jogs that queued up meanwhile
                    deltas = {self._joint(command): float(command["delta"])}
                    while not self.commands.empty():
                        nxt = json.loads(self.commands.queue[0])
                        if nxt.get("cmd") != "jog":
                            break
                        self.commands.get_nowait()
                        name = self._joint(nxt)
                        deltas[name] = deltas.get(name, 0.0) + float(nxt["delta"])
                    msg = self._jog(deltas)
                else:
                    msg = self._run(command)
            except Exception as error:
                msg = f"error: {error}"
                self.node.get_logger().error(f"arm jog: {text!r}: {error}")
            self._publish(msg)

    # ------------------------------------------------------------ commands
    def _ensure(self, legs=None):
        from ..manipulation.arm_controller import shared_controller      # shared with pick_place
        legs = legs or self.legs
        self.arm = shared_controller(self.client, legs=legs)
        self.legs = legs
        if not self.arm.enabled:
            self.arm.start()
            self.home = dict(self.arm.target)

    def _run(self, command):
        cmd = command.get("cmd")
        if cmd == "start":
            self._ensure(command.get("legs"))
            return f"arms controlled, holding the current pose (legs: {self.legs})"
        if cmd == "stop":
            if self.arm is not None:
                self.arm.stop()
            return "released"
        if cmd == "home":
            self._ensure()
            self.arm.move_joints(dict(self.home), duration=3.0)
            return "home"
        if cmd == "pose":
            from ..manipulation.arm_ik import joint_names
            side, name = command["side"], command.get("name", "calib")
            q, pitch = PRESETS[name][side]
            self._ensure()
            goals = dict(zip(joint_names(side), q))
            goals.update({HEAD["pitch"][0]: pitch, HEAD["yaw"][0]: 0.0})
            self.arm.move_joints(goals, duration=3.0)
            return f"pose {name} ({side})"
        raise ValueError(f"unknown cmd {cmd!r}")

    def _jog(self, deltas):
        self._ensure()
        goals = {}
        for name, delta in deltas.items():
            lo, hi = self._limits(name)
            goals[name] = min(hi, max(lo, self.arm.target[name] + delta))
        self.arm.move_joints(goals)
        return "jog " + ", ".join(f"{n} {v:+.3f}" for n, v in goals.items())

    @staticmethod
    def _joint(command):
        side, joint = command["side"], command["joint"]
        if side == "head":
            return HEAD[joint][0]
        if side not in ("left", "right"):
            raise ValueError("side must be left / right / head")
        return f"{side}_{joint}_joint"

    @staticmethod
    def _limits(name):
        for joint, lo, hi in HEAD.values():
            if name == joint:
                return lo, hi
        from ..manipulation.arm_ik import ARM_JOINTS, LIMITS
        side, joint = name.split("_", 1)
        return tuple(LIMITS[side][ARM_JOINTS.index(joint.removesuffix("_joint"))])

    def _publish(self, msg_text):
        state = {"enabled": bool(self.arm is not None and self.arm.enabled), "legs": self.legs, "msg": msg_text,
                 "q": {n: round(v, 4) for n, v in (self.arm.target.items() if self.arm else [])}}
        msg = String()
        msg.data = json.dumps(state)
        self.publisher.publish(msg)
