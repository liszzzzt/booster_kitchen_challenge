"""Revo2 dexterous hand on Booster T2 — one API, three backends.

    hand = SdkRevo2Hand(robot.loco_client, "right")      # real T2 (ControlDexterousHand)
    hand = RosRevo2Hand("right")                         # Booster Studio + robot_t2_revo2 + revo2_hand extension
    hand = MujocoRevo2Hand(model, data, "right")         # standalone MuJoCo (models/*.xml)
    hand = make_hand(robot, "right")                     # SDK if GetHands() reports hands, else ROS topic

    hand.open()
    hand.preset("power")              # or "pinch", "point", "open"
    hand.set(index=0.8, thumb_flex=0.5)  # 0.0 = fully open, 1.0 = fully closed, per DOF
    hand.state()                      # -> {dof: closure 0..1}

Revo2 has 6 motors per hand.  DOF names used everywhere in this folder:
    thumb_rotation  thumb_flex  index  middle  ring  pinky
In MuJoCo they map to <side>_hand_<dof> position actuators (built by tools/build_sim_models.py).

On the robot the hand is driven through booster_robotics_sdk_python
    B1LocoClient.ControlDexterousHand([DexterousFingerParameter(seq, angle, force, speed)...],
                                      HandIndex.kRightHand, BoosterHandType.kRevoHand)
boosteros' BoosterRobot already owns a B1LocoClient (robot.loco_client), so no extra client is needed.

!!! The seq order and angle direction for kRevoHand are NOT documented by Booster (the SDK header
!!! only documents the Inspire RH56: seq 0..5 = little, ring, middle, index, thumb bend, thumb
!!! rotation; angle 1000 = open, 0 = closed).  DEFAULT_SEQ / ANGLE_OPEN below follow that Inspire
!!! convention and MUST be confirmed with tools/probe_revo2_mapping.py on the real hand before use.
"""
from __future__ import annotations

import json
import threading
import time
from typing import Dict, Mapping, Optional

DOFS = ("thumb_rotation", "thumb_flex", "index", "middle", "ring", "pinky")

# 0 = open, 1 = closed  (thumb_rotation 1 = thumb swung in front of the palm, opposing the fingers)
PRESETS: Dict[str, Dict[str, float]] = {
    "open":  dict(thumb_rotation=0.0, thumb_flex=0.0, index=0.0, middle=0.0, ring=0.0, pinky=0.0),
    # thumb pre-rotated, fingers open: use before approaching an object
    "ready": dict(thumb_rotation=0.9, thumb_flex=0.0, index=0.0, middle=0.0, ring=0.0, pinky=0.0),
    "power": dict(thumb_rotation=0.9, thumb_flex=0.75, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
    "pinch": dict(thumb_rotation=0.75, thumb_flex=0.55, index=0.6, middle=0.0, ring=0.0, pinky=0.0),
    "point": dict(thumb_rotation=0.5, thumb_flex=0.8, index=0.0, middle=1.0, ring=1.0, pinky=1.0),
    "fist":  dict(thumb_rotation=0.3, thumb_flex=0.8, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
}


def _clip01(v: float) -> float:
    return 0.0 if v < 0.0 else 1.0 if v > 1.0 else float(v)


class Revo2HandBase:
    side: str

    def __init__(self, side: str):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.side = side
        self._target = dict(PRESETS["open"])

    # ---- public API -------------------------------------------------------------------------
    def set(self, closure: Optional[Mapping[str, float]] = None, **kw: float) -> None:
        """Set per-DOF closure (0 open .. 1 closed). Unspecified DOFs keep their last target."""
        upd = dict(closure or {}, **kw)
        for k, v in upd.items():
            if k not in DOFS:
                raise KeyError(f"unknown DOF {k!r}; valid: {DOFS}")
            self._target[k] = _clip01(v)
        self._send(dict(self._target))

    def preset(self, name: str, scale: float = 1.0) -> None:
        """Apply a named preset; scale<1 closes only part of the way (e.g. 0.5 for a pre-shape)."""
        p = PRESETS[name]
        self.set({k: v * scale for k, v in p.items()})

    def open(self) -> None:
        self.preset("open")

    def close(self, scale: float = 1.0) -> None:
        self.preset("power", scale)

    def target(self) -> Dict[str, float]:
        return dict(self._target)

    # ---- backend ----------------------------------------------------------------------------
    def _send(self, closure: Dict[str, float]) -> None:
        raise NotImplementedError

    def state(self) -> Dict[str, float]:
        raise NotImplementedError


# =============================================================================================
# MuJoCo backend
# =============================================================================================
class MujocoRevo2Hand(Revo2HandBase):
    """Writes data.ctrl of the <side>_hand_<dof> position actuators (see models/*.xml)."""

    def __init__(self, model, data, side: str = "right"):
        super().__init__(side)
        import mujoco  # local import: the SDK backend must not need mujoco
        self.m, self.d = model, data
        self._act = {}
        self._jnt = {}
        for dof in DOFS:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{side}_hand_{dof}")
            if aid < 0:
                raise KeyError(f"actuator {side}_hand_{dof} missing — build the model with tools/build_sim_models.py")
            self._act[dof] = aid
            self._jnt[dof] = model.actuator_trnid[aid, 0]

    def _send(self, closure: Dict[str, float]) -> None:
        for dof, c in closure.items():
            a = self._act[dof]
            lo, hi = self.m.actuator_ctrlrange[a]
            self.d.ctrl[a] = lo + c * (hi - lo)

    def state(self) -> Dict[str, float]:
        out = {}
        for dof, a in self._act.items():
            j = self._jnt[dof]
            lo, hi = self.m.jnt_range[j]
            out[dof] = _clip01((self.d.qpos[self.m.jnt_qposadr[j]] - lo) / (hi - lo))
        return out


# =============================================================================================
# Booster SDK backend (real robot, or Booster Studio virtual robot if it simulates the hand)
# =============================================================================================
# seq index on the wire for every DOF — Inspire convention, VERIFY with tools/probe_revo2_mapping.py
DEFAULT_SEQ = dict(pinky=0, ring=1, middle=2, index=3, thumb_flex=4, thumb_rotation=5)
ANGLE_OPEN = 1000   # Inspire convention: 1000 = open, 0 = closed. Set to 0 if Revo2 is the other way round.
ANGLE_CLOSED = 0


class SdkRevo2Hand(Revo2HandBase):
    def __init__(self, loco_client, side: str = "right", *,
                 seq_map: Optional[Mapping[str, int]] = None,
                 angle_open: int = ANGLE_OPEN, angle_closed: int = ANGLE_CLOSED,
                 force: int = 500, speed: int = 500, subscribe_state: bool = True,
                 check_hands: bool = True, virtual_robot_name: Optional[str] = None):
        super().__init__(side)
        import booster_robotics_sdk_python as b
        self.b = b
        self.client = loco_client
        self.seq_map = dict(seq_map or DEFAULT_SEQ)
        self.angle_open, self.angle_closed = angle_open, angle_closed
        self.force, self.speed = force, speed
        self.hand_index = b.HandIndex.kLeftHand if side == "left" else b.HandIndex.kRightHand
        self.hand_type = b.BoosterHandType.kRevoHand
        self._state_raw: Dict[int, int] = {}
        self._lock = threading.Lock()
        self.hands_info: Optional[str] = None
        if check_hands:
            try:
                self.hands_info = loco_client.GetHands().to_json_str()
            except Exception as e:  # virtual robot without hands, old firmware ...
                self.hands_info = f"GetHands failed: {e}"
        self._sub = None
        if subscribe_state:
            try:
                self._sub = b.B1LowHandDataSubscriber(self._on_hand_data)
                if virtual_robot_name:
                    self._sub.InitChannelWithName(virtual_robot_name)
                else:
                    self._sub.InitChannel()
            except Exception:
                self._sub = None

    @classmethod
    def from_booster_robot(cls, robot, side: str = "right", **kw) -> "SdkRevo2Hand":
        """robot = boosteros.robots.booster.BoosterRobot (what t2_robot_side/src/robot/t2.py creates)."""
        kw.setdefault("virtual_robot_name", getattr(robot, "virtual_robot_name", None) or None)
        return cls(robot.loco_client, side, **kw)

    # closure -> vendor angle
    def _angle(self, c: float) -> int:
        return int(round(self.angle_open + c * (self.angle_closed - self.angle_open)))

    def _closure(self, angle: int) -> float:
        span = self.angle_closed - self.angle_open
        return _clip01((angle - self.angle_open) / span) if span else 0.0

    def _send(self, closure: Dict[str, float]) -> None:
        P = self.b.DexterousFingerParameter
        params = [P(self.seq_map[d], self._angle(c), self.force, self.speed) for d, c in closure.items()]
        params.sort(key=lambda p: p.seq)
        self.client.ControlDexterousHand(params, self.hand_index, self.hand_type)

    def send_raw(self, seq: int, angle: int, force: Optional[int] = None, speed: Optional[int] = None) -> None:
        """Send a single seq/angle — used by the mapping probe."""
        p = self.b.DexterousFingerParameter(seq, int(angle), force or self.force, speed or self.speed)
        self.client.ControlDexterousHand([p], self.hand_index, self.hand_type)

    def _on_hand_data(self, msg) -> None:
        try:
            if int(msg.hand_index) != int(self.hand_index):
                return
            with self._lock:
                for p in msg.hand_data:
                    self._state_raw[int(p.seq)] = int(p.angle)
        except Exception:
            pass

    def raw_state(self) -> Dict[int, int]:
        with self._lock:
            return dict(self._state_raw)

    def state(self) -> Dict[str, float]:
        raw = self.raw_state()
        return {d: self._closure(raw[s]) for d, s in self.seq_map.items() if s in raw}

    def wait_until(self, closure: Mapping[str, float], tol: float = 0.1, timeout: float = 2.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self.state()
            if s and all(abs(s.get(k, 9) - v) < tol for k, v in closure.items()):
                return True
            time.sleep(0.05)
        return False


# =============================================================================================
# Booster Studio backend: ROS 2 topic of the revo2_hand physics extension (studio_assets/)
# =============================================================================================
class RosRevo2Hand(Revo2HandBase):
    """Publishes std_msgs/String JSON on /revo2/hand_cmd; reads /revo2/hand_state.

    Works wherever ROS 2 reaches the virtual robot (agent inside the container, T2WS sidecar).
    Pass an existing rclpy node to reuse it (its executor must spin for state()); otherwise a
    private node with its own spin thread is created.
    """

    def __init__(self, side: str = "right", *, node=None, robot: Optional[str] = None,
                 command_topic: str = "/revo2/hand_cmd", state_topic: str = "/revo2/hand_state"):
        super().__init__(side)
        import rclpy
        from std_msgs.msg import String
        self._String = String
        self.robot = robot
        self._state: Dict[str, float] = {}
        self._lock = threading.Lock()
        self._executor = None
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            from rclpy.executors import SingleThreadedExecutor
            node = rclpy.create_node(f"revo2_hand_client_{side}")
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            threading.Thread(target=self._executor.spin, daemon=True).start()
        self.node = node
        self._pub = node.create_publisher(String, command_topic, 10)
        node.create_subscription(String, state_topic, self._on_state, 10)

    def wait_for_extension(self, timeout: float = 5.0) -> bool:
        """True once the revo2_hand extension subscribes to the command topic."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self._pub.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        return False

    def _send(self, closure: Dict[str, float]) -> None:
        cmd = {"side": self.side, "closure": closure}
        if self.robot:
            cmd["robot"] = self.robot
        msg = self._String()
        msg.data = json.dumps(cmd)
        self._pub.publish(msg)

    def _on_state(self, msg) -> None:
        try:
            st = json.loads(msg.data)
            if st.get("side") == self.side and (self.robot is None or st.get("robot") == self.robot):
                with self._lock:
                    self._state = dict(st.get("closure", {}))
        except Exception:
            pass

    def state(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._state)


def make_hand(robot, side: str = "right", **kw) -> Revo2HandBase:
    """Real robot (GetHands lists hands) -> SdkRevo2Hand, otherwise (Booster Studio) -> RosRevo2Hand."""
    try:
        import json as _json
        info = _json.loads(robot.loco_client.GetHands().to_json_str())
        if info.get("body", {}).get("hands"):
            return SdkRevo2Hand.from_booster_robot(robot, side, **kw)
    except Exception:
        pass
    return RosRevo2Hand(side)
