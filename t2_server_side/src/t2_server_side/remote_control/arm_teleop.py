#!/usr/bin/env python3
"""Keyboard jogging of the T2 arms and head (robot side: relay_bridges/arm_jog.py in the T2 agent).

    python3 arm_teleop.py            # T2WS, the T2 agent must be running; legs locked (default)
    python3 arm_teleop.py --legs walk    # legs stay with the walking controller (balances, may sway / step)

    1 / 2      right / left arm
    q a        shoulder_pitch  + / -        t g   wrist_pitch  + / -
    w s        shoulder_roll   + / -        y h   wrist_yaw    + / -
    e d        elbow_pitch     + / -        u j   wrist_roll   + / -
    r f        elbow_yaw       + / -
    i k        head pitch      + / -  (+ = down)     o l   head yaw  + / -
    [ ]        step size / 2, x 2   (start 0.05 rad)
    c          calibration pose of the selected arm (board on the back of the hand faces the head camera)
    z          back to the pose when jogging started
    x          release: back to Booster's control (lock: prepare = standing; walk: walking controller)
    p          print the commanded angles of the selected arm
    Esc / Ctrl-C   quit (the arm keeps holding its pose; press x first to release it)

Hand-eye calibration: c, then small jogs (wrist pitch/yaw/roll, elbow) between captures so the board
moves around the image and tilts both ways; keep it facing the camera (collect.py rejects > 50 deg).
"""
import json
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

KEYS = {
    "q": ("shoulder_pitch", 1), "a": ("shoulder_pitch", -1),
    "w": ("shoulder_roll", 1), "s": ("shoulder_roll", -1),
    "e": ("elbow_pitch", 1), "d": ("elbow_pitch", -1),
    "r": ("elbow_yaw", 1), "f": ("elbow_yaw", -1),
    "t": ("wrist_pitch", 1), "g": ("wrist_pitch", -1),
    "y": ("wrist_yaw", 1), "h": ("wrist_yaw", -1),
    "u": ("wrist_roll", 1), "j": ("wrist_roll", -1),
}
HEAD_KEYS = {"i": ("pitch", 1), "k": ("pitch", -1), "o": ("yaw", 1), "l": ("yaw", -1)}


class Teleop(Node):
    def __init__(self):
        super().__init__("t2_arm_teleop")
        self.publisher = self.create_publisher(String, "/t2/arm/jog", 10)
        self.create_subscription(String, "/t2/arm/state", self._on_state, 10)
        self.state = None

    def _on_state(self, msg):
        self.state = json.loads(msg.data)
        print(f"\r  {self.state['msg']}" + " " * 10, flush=True)

    def send(self, **command):
        msg = String()
        msg.data = json.dumps(command)
        self.publisher.publish(msg)


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--legs", choices=["lock", "walk"], default="lock",
                    help="lock: legs held stiff in the standing posture (no sway); walk: walking controller")
    a = ap.parse_args()
    rclpy.init()
    node = Teleop()
    end = time.monotonic() + 15
    while node.publisher.get_subscription_count() == 0:
        if time.monotonic() > end:
            sys.exit("no subscriber on /t2/arm/jog: is the T2 agent (with arm_jog) running?")
        rclpy.spin_once(node, timeout_sec=0.1)
    print(__doc__.split("\n\n")[1])
    node.send(cmd="start", legs=a.legs)
    side, step = "right", 0.05
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        import select
        print(f"arm: {side}, step {step:.3f} rad")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            k = sys.stdin.read(1)
            if not k or k == "\x1b":
                break
            if k in "12":
                side = "right" if k == "1" else "left"
                print(f"arm: {side}")
            elif k in KEYS:
                joint, sign = KEYS[k]
                node.send(cmd="jog", side=side, joint=joint, delta=sign * step)
            elif k in HEAD_KEYS:
                joint, sign = HEAD_KEYS[k]
                node.send(cmd="jog", side="head", joint=joint, delta=sign * step)
            elif k in "[]":
                step = min(0.4, max(0.005, step * (0.5 if k == "[" else 2.0)))
                print(f"step {step:.3f} rad")
            elif k == "c":
                node.send(cmd="pose", side=side, name="calib")
            elif k == "z":
                node.send(cmd="home")
            elif k == "x":
                node.send(cmd="stop")
            elif k == "p":
                q = (node.state or {}).get("q", {})
                print({n.replace(f"{side}_", "").replace("_joint", ""): v for n, v in q.items() if n.startswith(side)})
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
