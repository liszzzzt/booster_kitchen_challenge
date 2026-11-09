#!/usr/bin/env python3
"""Move T2's arms to Cartesian targets: IK (arm_ik.py) + boosteros upper-body joint control.

Works the same on the real robot and in Booster Studio (both go through booster-motion):
    walk mode -> upper_body_control(True) -> set_joints(16 upper-body JointCommands) ...
                 -> upper_body_control(False)
Legs and waist stay with the locomotion controller (it keeps balance); this class only commands
head (held) + both arms.

    from boosteros.robots.booster import BoosterRobot
    from arm_controller import ArmController
    from arm_ik import orientation

    arm = ArmController(BoosterRobot())            # or pass an existing client (t2_robot_side: T2Robot.client)
    arm.start()                                    # walk mode + upper-body control on
    print(arm.palm_pose("right"))                  # where the palm is now (trunk frame)
    arm.move_to("right", [0.35, -0.20, -0.05], R=orientation("right", "side"))
    arm.move_line("right", [0.35, -0.20, 0.05])     # straight-line lift, keep orientation
    arm.stop()                                     # upper-body control off -> booster-motion's arm pose

Positions are in the trunk frame (x forward, y left, z up; origin = trunk body), see arm_ik.py.
Requires: numpy, boosteros (pip install boosteros) — and the Booster SDK it uses.
"""
from __future__ import annotations

import logging
import time

import numpy as np

try:                                    # as package: robot.manipulation.arm_controller
    from .arm_ik import ArmIK, joint_names
except ImportError:                     # as script / sys.path on this folder
    from arm_ik import ArmIK, joint_names

log = logging.getLogger("arm_controller")

# kp / kd per joint role, from boosteros/examples/booster/joint_upper_control_example.py
GAINS = {
    "head_yaw": (40.0, 2.0), "head_pitch": (40.0, 2.0),
    "shoulder_pitch": (20.0, 1.5), "shoulder_roll": (30.0, 2.0),
    "elbow_pitch": (10.0, 0.5), "elbow_yaw": (10.0, 0.5),
    "wrist_pitch": (20.0, 1.5), "wrist_yaw": (30.0, 2.0), "wrist_roll": (10.0, 0.5),
}
UPPER_COUNT = 16          # T2: head 2 + left arm 7 + right arm 7 (first 16 of list_joints())


def gain(name: str, gains=GAINS):
    n = name.lower().removesuffix("_joint")
    for k, v in gains.items():
        if n.endswith(k):
            return v
    raise ValueError(f"no gain for joint {name!r}")


def min_jerk(s):
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


class IKError(RuntimeError):
    pass


class ArmController:
    def __init__(self, robot, *, point="palm", rate_hz=50.0, max_vel=0.8, gains=None,
                 gain_scale=1.0, ensure_walk=True):
        """robot     : boosteros BoosterRobot (already constructed)
        point     : which hand point the targets refer to: "palm" | "grasp" | "wrist"
        max_vel   : joint speed limit used to pick move durations [rad/s]
        gain_scale: multiply all kp/kd (the boosteros example gains are fairly soft;
                    raise to ~2-3 if the arm sags under gravity, check carefully on the real robot)
        """
        from boosteros.types import JointCommand            # imported here so arm_ik stays SDK-free
        self._JC = JointCommand
        self.robot = robot
        self.dt = 1.0 / rate_hz
        self.max_vel = max_vel
        self.gains = gains or GAINS
        self.gain_scale = gain_scale
        self.ensure_walk = ensure_walk
        self.ik = {s: ArmIK(s, point) for s in ("left", "right")}
        self.names: list[str] = []
        self.target: dict[str, float] = {}
        self.enabled = False

    # ------------------------------------------------------------ lifecycle
    def start(self, settle=1.0):
        r = self.robot
        if self.ensure_walk and r.get_mode() != "walk":
            log.info("switching to walk mode")
            r.set_mode("walk")
            time.sleep(settle)
        names = [j.name for j in r.list_joints()]
        if len(names) != 31:
            raise RuntimeError(f"expected T2 with 31 joints, got {len(names)}")
        self.names = names[:UPPER_COUNT]
        for s in ("left", "right"):
            missing = set(joint_names(s)) - set(self.names)
            if missing:
                raise RuntimeError(f"upper-body joints missing {missing}; got {self.names}")
        cur = self.joint_positions()
        self.target = {n: cur[n] for n in self.names}        # hold the current pose first
        r.upper_body_control(True)
        self.enabled = True
        self._send()
        log.info("upper-body control enabled")
        return self

    def stop(self):
        if self.enabled:
            try:
                self.robot.upper_body_control(False)
            finally:
                self.enabled = False
            log.info("upper-body control disabled")

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    # ------------------------------------------------------------ state
    def joint_positions(self) -> dict[str, float]:
        return {s.name: float(s.position) for s in self.robot.get_joint_states().joints
                if s.position is not None}

    def arm_q(self, side, measured=True) -> np.ndarray:
        """7 arm joint angles (IK order): measured, or the currently commanded target."""
        src = self.joint_positions() if measured else self.target
        return np.array([src[n] for n in joint_names(side)])

    def palm_pose(self, side, measured=True):
        """(position, rotation) of the hand point in the trunk frame."""
        return self.ik[side].fk(self.arm_q(side, measured))

    # ------------------------------------------------------------ low level
    def _send(self):
        cmds = []
        for n in self.names:
            kp, kd = gain(n, self.gains)
            cmds.append(self._JC(name=n, position=float(self.target[n]),
                                 kp=kp * self.gain_scale, kd=kd * self.gain_scale))
        self.robot.set_joints(cmds)

    def _require(self):
        if not self.enabled:
            raise RuntimeError("call start() first")

    def move_joints(self, goals: dict[str, float], duration=None):
        """Min-jerk joint-space move of any subset of the 16 upper-body joints."""
        self._require()
        start = {n: self.target[n] for n in goals}
        dmax = max((abs(goals[n] - start[n]) for n in goals), default=0.0)
        T = max(duration or 0.0, dmax / self.max_vel * 1.875, 0.2)   # 1.875 = min-jerk peak/avg speed
        steps = max(1, int(round(T / self.dt)))
        t0 = time.monotonic()
        for k in range(1, steps + 1):
            a = min_jerk(k / steps)
            for n in goals:
                self.target[n] = start[n] + a * (goals[n] - start[n])
            self._send()
            time.sleep(max(0.0, t0 + k * self.dt - time.monotonic()))
        return T

    def move_arm_q(self, side, q7, duration=None):
        return self.move_joints(dict(zip(joint_names(side), map(float, q7))), duration)

    # ------------------------------------------------------------ Cartesian
    def solve(self, side, pos, R=None, axis=None, **kw):
        """IK only (nothing is sent). Seeded from the commanded posture. Raises IKError."""
        q0 = self.arm_q(side, measured=False) if self.target else None
        q, info = self.ik[side].solve(pos, R=R, axis=axis, q0=q0, **kw)
        if not info["ok"]:
            raise IKError(f"{side}: target {np.round(pos, 3)} not reachable "
                          f"(pos err {info['pos_err']*1000:.0f} mm, rot err {np.degrees(info['rot_err']):.0f} deg)")
        return q, info

    def move_to(self, side, pos, R=None, axis=None, duration=None, **kw):
        """Joint-space move to an IK solution (the hand path is curved; fine in free space)."""
        q, info = self.solve(side, pos, R=R, axis=axis, **kw)
        self.move_arm_q(side, q, duration)
        return q

    def move_line(self, side, pos, R=None, speed=0.10, keep_orientation=True):
        """Straight-line move of the hand point at `speed` m/s (approach / lift / retreat).
        Orientation: R if given, else the current commanded orientation (keep_orientation)."""
        self._require()
        q = self.arm_q(side, measured=False)
        p0, R0 = self.ik[side].fk(q)
        R = R if R is not None else (R0 if keep_orientation else None)
        pos = np.asarray(pos, float)
        dist = float(np.linalg.norm(pos - p0))
        n = max(1, int(np.ceil(dist / (speed * self.dt))))
        # solve the whole path first, then execute (no half-finished moves)
        path = []
        ik = self.ik[side]
        for k in range(1, n + 1):
            a = min_jerk(k / n)
            q, info = ik.solve(p0 + a * (pos - p0), R=R, q0=q, restarts=0, iters=50)
            if not info["ok"]:
                raise IKError(f"{side}: line path leaves the workspace at {np.round(p0 + a*(pos-p0), 3)}")
            if np.max(np.abs(q - (path[-1] if path else self.arm_q(side, False)))) > 0.2:
                raise IKError(f"{side}: line path needs a joint jump (singularity / limit)")
            path.append(q)
        names = joint_names(side)
        t0 = time.monotonic()
        for k, q in enumerate(path, 1):
            self.target.update(zip(names, map(float, q)))
            self._send()
            time.sleep(max(0.0, t0 + k * self.dt - time.monotonic()))
        return path[-1]
