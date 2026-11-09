"""revo2_hand — Booster Studio physics extension that drives the Revo2 fingers of robot_t2_revo2.

Installed into the virtual-robot container at
    /usr/local/booster_robot/booster_robocup_sim/physics/extensions/revo2_hand/
and registered in physics/extensions/builtin.py (see install_in_container.sh).
install_in_container.sh also patches host.py so it is active in every scene (idle without hands);
a scene may still declare it explicitly: {"point": "physics", "name": "revo2_hand", "params": {...}}.

What it does
  * finds the hands of robot_t2_revo2: body <robot>_<side>_revo2_hand and the 6 driven joints
    <robot>_<side>_{thumb_metacarpal,thumb_proximal,index_proximal,middle_proximal,ring_proximal,pinky_proximal}_joint
    (= dof thumb_rotation thumb_flex index middle ring pinky). The asset has NO finger actuators,
    because Studio only accepts robot assets with exactly the 31 T2 actuators;
  * before every physics step applies a PD torque on those joints through data.qfrc_applied
    (target = closure 0=open .. 1=closed, slew-rate limited like a real Revo2 stroke, torque clamped
    to the Revo2 joint effort); distal joints follow through the asset's equality couplings;
  * ROS 2 (rclpy, std_msgs/String with JSON):
      subscribe  command_topic (default /revo2/hand_cmd)
          {"side": "right", "preset": "power"}                      presets: open ready power pinch point fist
          {"side": "right", "preset": "power", "scale": 0.5}
          {"side": "left",  "closure": {"index": 0.8, "thumb_flex": 0.5}}
          {"side": "both",  "index": 1.0}                           dof keys may also be top level
          optional "robot": "robot1"  (default: every robot that has Revo2 hands)
      publish    state_topic (default /revo2/hand_state), state_rate_hz (default 20)
          {"robot": "robot1", "side": "right", "closure": {...}, "target": {...}, "time": sim_time}
      publish    contact_topic (default /revo2/contacts), same rate; per hand, contacts between the hand and
                 anything that is not the robot itself (forces in N, from mj_contactForce):
          {"robot", "side", "time", "grip_fn", "friction_ft", "max_util", "max_slip", "sliding",
           "fingers": {"thumb"|"index"|"middle"|"ring"|"pinky"|"palm": {"fn", "ft", "util", "slip", "n"}},
           "objects": {"<body>": {"fn", "ft", "mu", "max_util", "max_slip", "n", "net_force": [fx, fy, fz]}}}
          fn = sum of normal forces, ft = sum of |tangential friction forces|, mu = sliding friction
          coefficient of the contact pair, util = ft / (mu * fn) of the worst contact,
          slip = relative tangential speed hand<->object at the contact [m/s] (max over contacts),
          sliding = any contact with slip > 5 mm/s, net_force = total hand force on that object (world).
          NOTE: Studio scenes use MuJoCo's pyramidal friction cone; there the solver often puts a contact
          force on a cone edge, so util = 1.0 also happens while static. Use slip / sliding for slipping.
If the scene has no Revo2 hands the extension stays idle.
"""
from __future__ import annotations

import json
import re
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

DOFS = ("thumb_rotation", "thumb_flex", "index", "middle", "ring", "pinky")
PRESETS: Dict[str, Dict[str, float]] = {
    "open":  dict(thumb_rotation=0.0, thumb_flex=0.0, index=0.0, middle=0.0, ring=0.0, pinky=0.0),
    "ready": dict(thumb_rotation=0.9, thumb_flex=0.0, index=0.0, middle=0.0, ring=0.0, pinky=0.0),
    "power": dict(thumb_rotation=0.9, thumb_flex=0.75, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
    "pinch": dict(thumb_rotation=0.75, thumb_flex=0.55, index=0.6, middle=0.0, ring=0.0, pinky=0.0),
    "point": dict(thumb_rotation=0.5, thumb_flex=0.8, index=0.0, middle=1.0, ring=1.0, pinky=1.0),
    "fist":  dict(thumb_rotation=0.3, thumb_flex=0.8, index=1.0, middle=1.0, ring=1.0, pinky=1.0),
}
DOF_JOINTS = ("thumb_metacarpal_joint", "thumb_proximal_joint", "index_proximal_joint",
              "middle_proximal_joint", "ring_proximal_joint", "pinky_proximal_joint")
KP = np.array([2.0, 4.0, 4.0, 4.0, 4.0, 4.0])        # Nm/rad
KD = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05])  # Nm s/rad
TAU_MAX = np.array([0.5, 1.1, 2.0, 2.0, 2.0, 2.0])   # Nm, Revo2 joint effort (URDF)
_HAND_RE = re.compile(r"^(?P<robot>robot\d+)_(?P<side>left|right)_revo2_hand$")
_LOG = "[revo2_hand]"
SNAP_DISTANCE = 0.03   # m; weld drift above this means the robot was teleported
FINGER_KEYS = ("thumb", "index", "middle", "ring", "pinky")   # anything else in the hand = "palm"
SLIP_SPEED = 0.005     # m/s; relative tangential speed above this at a contact = sliding


@dataclass(frozen=True)
class Revo2HandConfig:
    command_topic: str = "/revo2/hand_cmd"
    state_topic: str = "/revo2/hand_state"
    state_rate_hz: float = 20.0
    contact_topic: str = "/revo2/contacts"
    max_rate: float = 2.0          # closure units per second (full stroke ~0.5 s)
    ros: bool = True
    dds_profile: str = "/opt/booster/BoosterRos2/fastdds_profile.xml"   # same as ros_bridge_server


def parse_revo2_hand_config(params: Dict[str, Any]) -> Revo2HandConfig:
    allowed = {"command_topic", "state_topic", "contact_topic", "state_rate_hz", "max_rate", "ros", "dds_profile"}
    unknown = sorted(set(params) - allowed)
    if unknown:
        raise ValueError(f"Unknown revo2_hand param(s): {unknown}")
    ros = str(params.get("ros", "true")).strip().lower() not in ("0", "false", "no", "off")
    cfg = Revo2HandConfig(
        command_topic=str(params.get("command_topic", "/revo2/hand_cmd")),
        state_topic=str(params.get("state_topic", "/revo2/hand_state")),
        contact_topic=str(params.get("contact_topic", "/revo2/contacts")),
        state_rate_hz=float(params.get("state_rate_hz", 20.0)),
        max_rate=float(params.get("max_rate", 2.0)),
        ros=ros,
        dds_profile=str(params.get("dds_profile", "/opt/booster/BoosterRos2/fastdds_profile.xml")),
    )
    if cfg.state_rate_hz <= 0 or cfg.max_rate <= 0:
        raise ValueError("revo2_hand state_rate_hz and max_rate must be > 0")
    return cfg


class _Hand:
    """One hand of one robot: driven-joint addresses in DOFS order."""

    def __init__(self, robot: str, side: str):
        self.robot, self.side = robot, side
        self.jid = np.full(len(DOFS), -1, dtype=np.int64)
        self.qadr = np.zeros(len(DOFS), dtype=np.int64)
        self.dadr = np.zeros(len(DOFS), dtype=np.int64)
        self.lo = np.zeros(len(DOFS))
        self.hi = np.ones(len(DOFS))
        self.target = np.zeros(len(DOFS))   # commanded closure
        self.current = np.zeros(len(DOFS))  # slew-limited closure actually sent
        # weld re-snap (hand body <-> wrist), filled by _bind when the bodies exist
        self.hand_body = self.wrist_body = self.parent_body = -1
        self.slide_qadr = self.slide_dadr = None   # 3 slide joints (tx, ty, tz)
        self.ball_qadr = self.ball_dadr = -1       # ball joint
        self.body_key: Dict[int, str] = {}        # hand-subtree body id -> finger key (contact report)

    @property
    def complete(self) -> bool:
        return bool(np.all(self.jid >= 0))


class Revo2HandRuntime:
    def __init__(self, config: Revo2HandConfig):
        self.cfg = config
        self.model = None
        self.data = None
        self.hands: Dict[Tuple[str, str], _Hand] = {}
        self._lock = threading.Lock()
        self._pending: List[dict] = []
        self._last_state_pub = -1e9
        self._ros_node = None
        self._ros_pub = None
        self._ros_contact_pub = None
        self._ros_thread: Optional[threading.Thread] = None
        self._ros_executor = None
        self._ros_context = None
        self._ros_started = False

    # ---------------------------------------------------------------- lifecycle
    def initialize(self, model: Any, data: Any) -> None:
        self._bind(model, data)
        if self.hands and self.cfg.ros and not self._ros_started:
            self._start_ros()

    def on_model_recompiled(self, old_model: Any, old_data: Any, new_model: Any, new_data: Any) -> None:
        keep = {k: (h.target.copy(), h.current.copy()) for k, h in self.hands.items()}
        self._bind(new_model, new_data)
        for k, (t, c) in keep.items():
            if k in self.hands:
                self.hands[k].target[:] = t
                self.hands[k].current[:] = c
        if self.hands and self.cfg.ros and not self._ros_started:
            self._start_ros()

    def on_reset(self) -> None:
        for h in self.hands.values():
            h.target[:] = 0.0
            h.current[:] = 0.0
            self._snap_hand(h)

    def on_body_pose_changed(self, model: Any, data: Any, body_id: int) -> None:
        for h in self.hands.values():
            self._snap_hand(h)

    def before_step(self, profiler: Any = None) -> None:
        if not self.hands or self.data is None:
            return
        self._drain_commands()
        step = self.cfg.max_rate * float(self.model.opt.timestep)
        d = self.data
        xpos = d.xpos
        for h in self.hands.values():
            # robot teleported (spawn hold, reset, drag) -> put the hand back on the wrist at once
            if h.hand_body >= 0 and np.linalg.norm(xpos[h.hand_body] - xpos[h.wrist_body]) > SNAP_DISTANCE:
                self._snap_hand(h)
            h.current += np.clip(h.target - h.current, -step, step)
            q_des = h.lo + h.current * (h.hi - h.lo)
            tau = KP * (q_des - d.qpos[h.qadr]) - KD * d.qvel[h.dadr]
            d.qfrc_applied[h.dadr] = np.clip(tau, -TAU_MAX, TAU_MAX)

    def post_step(self, profiler: Any = None) -> None:
        if self._ros_pub is None or self.data is None:
            return
        now = time.monotonic()
        if now - self._last_state_pub < 1.0 / self.cfg.state_rate_hz:
            return
        self._last_state_pub = now
        try:
            from std_msgs.msg import String
            for msg in self.state():
                s = String()
                s.data = json.dumps(msg)
                self._ros_pub.publish(s)
            if self._ros_contact_pub is not None:
                for msg in self.contacts():
                    s = String()
                    s.data = json.dumps(msg)
                    self._ros_contact_pub.publish(s)
        except Exception as exc:  # never break the physics loop
            print(f"{_LOG} state publish failed: {exc}")

    def dispose(self) -> None:
        if self.data is not None:
            for h in self.hands.values():
                self.data.qfrc_applied[h.dadr] = 0.0
        try:
            if self._ros_executor is not None:
                self._ros_executor.shutdown(timeout_sec=1.0)
            if self._ros_node is not None:
                self._ros_node.destroy_node()
            if self._ros_context is not None:
                import rclpy
                rclpy.try_shutdown(context=self._ros_context)
        except Exception:
            pass
        self._ros_node = self._ros_pub = self._ros_contact_pub = self._ros_executor = self._ros_context = None

    # ---------------------------------------------------------------- public helpers
    def submit(self, command: dict) -> None:
        """Queue one command dict (same format as the ROS JSON). Thread-safe."""
        with self._lock:
            self._pending.append(command)

    def state(self) -> List[dict]:
        out = []
        q = self.data.qpos
        t = float(self.data.time)
        for h in self.hands.values():
            closure = np.clip((q[h.qadr] - h.lo) / np.maximum(h.hi - h.lo, 1e-9), 0.0, 1.0)
            out.append({
                "robot": h.robot, "side": h.side, "time": round(t, 4),
                "closure": {d: round(float(v), 3) for d, v in zip(DOFS, closure)},
                "target": {d: round(float(v), 3) for d, v in zip(DOFS, h.target)},
            })
        return out

    def contacts(self) -> List[dict]:
        """Contact / friction report per hand (see module docstring). Pure read of mjData."""
        import mujoco
        m, d = self.model, self.data
        out = []
        if m is None or d is None:
            return out
        f6 = np.zeros(6)
        nv = int(m.nv)
        jp1, jp2 = np.zeros((3, nv)), np.zeros((3, nv))
        rep = {}
        for h in self.hands.values():
            rep[(h.robot, h.side)] = dict(
                fingers={k: dict(fn=0.0, ft=0.0, util=0.0, slip=0.0, n=0) for k in FINGER_KEYS + ("palm",)},
                objects={})
        owner = {}                                   # body id -> (hand key, finger key)
        for key, h in self.hands.items():
            for b, fk in h.body_key.items():
                owner[b] = (key, fk)
        for i in range(int(d.ncon)):
            c = d.contact[i]
            b1, b2 = int(m.geom_bodyid[c.geom1]), int(m.geom_bodyid[c.geom2])
            for hb, ob, sign in ((b1, b2, 1.0), (b2, b1, -1.0)):
                if hb not in owner or ob in owner:
                    continue
                key, fk = owner[hb]
                h = self.hands[key]
                root = int(m.body_rootid[ob])
                if root == int(m.body_rootid[h.hand_body]) and root != 0:
                    continue                         # the robot touching itself
                mujoco.mj_contactForce(m, d, i, f6)
                fn = float(f6[0])
                if fn <= 1e-6:
                    continue
                ft = float(np.hypot(f6[1], f6[2]))
                mu = float(c.friction[0])
                util = ft / (mu * fn) if mu > 0 else 0.0
                fw = c.frame.reshape(3, 3).T @ f6[:3]   # force geom1 -> geom2, world frame
                # relative tangential velocity of the two bodies at the contact point
                mujoco.mj_jac(m, d, jp1, None, c.pos, hb)
                mujoco.mj_jac(m, d, jp2, None, c.pos, ob)
                vrel = (jp1 - jp2) @ d.qvel
                nrm = c.frame[:3]
                slip = float(np.linalg.norm(vrel - np.dot(vrel, nrm) * nrm))
                # force ON the other body: +fw if it is geom2 (hand is geom1), else -fw
                f_on_obj = fw if sign > 0 else -fw
                r = rep[key]
                fr = r["fingers"][fk]
                fr["fn"] += fn; fr["ft"] += ft; fr["n"] += 1; fr["util"] = max(fr["util"], util)
                fr["slip"] = max(fr["slip"], slip)
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, ob) or f"body{ob}"
                o = r["objects"].setdefault(name, dict(fn=0.0, ft=0.0, mu=mu, max_util=0.0, max_slip=0.0, n=0,
                                                       net_force=np.zeros(3)))
                o["fn"] += fn; o["ft"] += ft; o["n"] += 1; o["max_util"] = max(o["max_util"], util)
                o["max_slip"] = max(o["max_slip"], slip)
                o["net_force"] += f_on_obj
        t = round(float(d.time), 4)
        for (robot, side), r in rep.items():
            fingers = {k: {kk: (round(v, 4) if isinstance(v, float) else v) for kk, v in f.items()}
                       for k, f in r["fingers"].items()}
            objects = {n: dict(fn=round(o["fn"], 3), ft=round(o["ft"], 3), mu=round(o["mu"], 3),
                               max_util=round(o["max_util"], 3), max_slip=round(o["max_slip"], 4), n=o["n"],
                               net_force=[round(float(x), 3) for x in o["net_force"]])
                       for n, o in r["objects"].items()}
            mx = max((f["util"] for f in r["fingers"].values()), default=0.0)
            ms = max((f["slip"] for f in r["fingers"].values() if f["n"]), default=0.0)
            out.append(dict(robot=robot, side=side, time=t,
                            grip_fn=round(sum(f["fn"] for f in r["fingers"].values()), 3),
                            friction_ft=round(sum(f["ft"] for f in r["fingers"].values()), 3),
                            max_util=round(mx, 3), max_slip=round(ms, 4), sliding=bool(ms > SLIP_SPEED),
                            fingers=fingers, objects=objects))
        return out

    # ---------------------------------------------------------------- internals
    def _bind(self, model: Any, data: Any) -> None:
        import mujoco
        self.model, self.data = model, data
        hands: Dict[Tuple[str, str], _Hand] = {}
        for b in range(int(model.nbody)):
            m = _HAND_RE.match(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "")
            if not m:
                continue
            robot, side = m.group("robot"), m.group("side")
            h = hands.setdefault((robot, side), _Hand(robot, side))
            for k, jn in enumerate(DOF_JOINTS):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_{side}_{jn}")
                if jid < 0:
                    continue
                h.jid[k] = jid
                h.qadr[k] = int(model.jnt_qposadr[jid])
                h.dadr[k] = int(model.jnt_dofadr[jid])
                h.lo[k], h.hi[k] = (float(v) for v in model.jnt_range[jid])
        self.hands = {k: h for k, h in hands.items() if h.complete}
        for (robot, side), h in self.hands.items():
            hb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_{side}_revo2_hand")
            wb = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{robot}_{side}_wrist_roll_link")
            if hb < 0 or wb < 0:
                continue
            j = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot}_{side}_revo2_hand_{n}")
                 for n in ("tx", "ty", "tz", "rot")]
            if min(j) < 0:
                continue
            h.hand_body, h.wrist_body, h.parent_body = hb, wb, int(model.body_parentid[hb])
            h.body_key = {}
            for b in range(int(model.nbody)):
                a = b
                while a > 0 and a != hb:
                    a = int(model.body_parentid[a])
                if a == hb:
                    bn = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or ""
                    h.body_key[b] = next((k for k in FINGER_KEYS if f"_{k}_" in bn), "palm")
            h.slide_qadr = np.array([model.jnt_qposadr[k] for k in j[:3]])
            h.slide_dadr = np.array([model.jnt_dofadr[k] for k in j[:3]])
            h.ball_qadr, h.ball_dadr = int(model.jnt_qposadr[j[3]]), int(model.jnt_dofadr[j[3]])
        if self.hands:
            print(f"{_LOG} bound hands: {sorted(self.hands)}")
        else:
            print(f"{_LOG} no Revo2 hands in this scene; idle")

    def _snap_hand(self, h: "_Hand") -> None:
        """Set the hand's 3 slide + ball joints so the hand body coincides with the wrist (weld relpose = identity)."""
        if h.hand_body < 0 or self.data is None:
            return
        import mujoco
        m, d = self.model, self.data
        mujoco.mj_kinematics(m, d)
        pp, pq = d.xpos[h.parent_body], d.xquat[h.parent_body]
        wp, wq = d.xpos[h.wrist_body], d.xquat[h.wrist_body]
        inv_pq = np.zeros(4); mujoco.mju_negQuat(inv_pq, pq)
        rel_p = np.zeros(3); mujoco.mju_rotVecQuat(rel_p, wp - pp, inv_pq)      # wrist in parent frame
        rel_q = np.zeros(4); mujoco.mju_mulQuat(rel_q, inv_pq, wq)
        p0, q0 = m.body_pos[h.hand_body], m.body_quat[h.hand_body]               # hand default in parent frame
        inv_q0 = np.zeros(4); mujoco.mju_negQuat(inv_q0, q0)
        slide = np.zeros(3); mujoco.mju_rotVecQuat(slide, rel_p - p0, inv_q0)
        ball = np.zeros(4); mujoco.mju_mulQuat(ball, inv_q0, rel_q)
        d.qpos[h.slide_qadr] = slide
        d.qpos[h.ball_qadr:h.ball_qadr + 4] = ball / np.linalg.norm(ball)
        d.qvel[h.slide_dadr] = 0.0
        d.qvel[h.ball_dadr:h.ball_dadr + 3] = 0.0

    def _drain_commands(self) -> None:
        with self._lock:
            pending, self._pending = self._pending, []
        for cmd in pending:
            try:
                self._apply(cmd)
            except Exception as exc:
                print(f"{_LOG} bad command {cmd!r}: {exc}")

    def _apply(self, cmd: dict) -> None:
        sides = cmd.get("side", "right")
        sides = ("left", "right") if sides in ("both", "all") else (str(sides),)
        robot = cmd.get("robot")
        upd: Dict[str, float] = {}
        if cmd.get("preset"):
            if cmd["preset"] not in PRESETS:
                raise ValueError(f"unknown preset {cmd['preset']!r}")
            scale = float(cmd.get("scale", 1.0))
            upd.update({k: v * scale for k, v in PRESETS[cmd["preset"]].items()})
        upd.update({k: float(v) for k, v in dict(cmd.get("closure", {})).items()})
        upd.update({k: float(cmd[k]) for k in DOFS if k in cmd})
        bad = [k for k in upd if k not in DOFS]
        if bad:
            raise ValueError(f"unknown dof(s) {bad}; valid: {DOFS}")
        matched = 0
        for (r, s), h in self.hands.items():
            if s in sides and (robot is None or robot == r):
                for k, v in upd.items():
                    h.target[DOFS.index(k)] = min(1.0, max(0.0, v))
                matched += 1
        if not matched:
            raise ValueError(f"no hand matches robot={robot} side={sides}")

    def _start_ros(self) -> None:
        """Own rclpy context + the same Fast DDS profile as Booster's ROS nodes.

        bs-phys-worker runs without FASTRTPS/FASTDDS_DEFAULT_PROFILES_FILE, while ros_bridge_server
        and the T2WS sidecar use a loopback-whitelisted profile. A participant created with the
        default profile is visible inside the container but not from T2WS, so the extension creates
        its own context with the Booster profile (env set only around init, then restored).
        """
        self._ros_started = True
        try:
            import os
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
            profile = self.cfg.dds_profile
            saved = {k: os.environ.get(k) for k in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")}
            use_profile = bool(profile) and os.path.isfile(profile)
            if use_profile:
                for k in saved:
                    os.environ[k] = profile
            try:
                ctx = rclpy.Context()
                rclpy.init(context=ctx)
                node = rclpy.create_node("revo2_hand_extension", context=ctx)
            finally:
                if use_profile:
                    for k, v in saved.items():
                        if v is None:
                            os.environ.pop(k, None)
                        else:
                            os.environ[k] = v
            node.create_subscription(String, self.cfg.command_topic, self._on_ros_cmd, 10)
            self._ros_pub = node.create_publisher(String, self.cfg.state_topic, 10)
            if self.cfg.contact_topic:
                self._ros_contact_pub = node.create_publisher(String, self.cfg.contact_topic, 10)
            ex = SingleThreadedExecutor(context=ctx)
            ex.add_node(node)
            self._ros_node, self._ros_executor, self._ros_context = node, ex, ctx
            self._ros_thread = threading.Thread(target=self._spin, name="revo2_hand_ros", daemon=True)
            self._ros_thread.start()
            print(f"{_LOG} ROS 2: listening on {self.cfg.command_topic}, publishing {self.cfg.state_topic}"
                  f" + {self.cfg.contact_topic or '(no contact topic)'}"
                  f" (DDS profile: {profile if use_profile else 'default'})")
        except Exception as exc:
            print(f"{_LOG} ROS 2 unavailable ({exc}); hands only respond to Revo2HandRuntime.submit()")

    def _spin(self) -> None:
        try:
            self._ros_executor.spin()
        except Exception as exc:
            print(f"{_LOG} ROS spin stopped: {exc}")

    def _on_ros_cmd(self, msg: Any) -> None:
        try:
            cmd = json.loads(msg.data)
            if not isinstance(cmd, dict):
                raise ValueError("command must be a JSON object")
            self.submit(cmd)
        except Exception as exc:
            print(f"{_LOG} ignored message {getattr(msg, 'data', msg)!r}: {exc}")


class Revo2HandPhysicsExtension:
    name = "revo2_hand"

    def __init__(self) -> None:
        self.runtime: Optional[Revo2HandRuntime] = None   # last activated runtime (for debugging)

    def parse_config(self, params: Dict[str, Any]) -> Revo2HandConfig:
        return parse_revo2_hand_config(params)

    def contribute(self, context: Any, config: Revo2HandConfig) -> None:
        runtime = Revo2HandRuntime(config)
        self.runtime = runtime
        context.lifecycle.on_initialize(runtime.initialize)
        context.lifecycle.on_before_step(runtime.before_step)
        context.lifecycle.on_post_step(runtime.post_step)
        context.lifecycle.on_reset(runtime.on_reset)
        context.lifecycle.on_model_recompiled(runtime.on_model_recompiled)
        context.lifecycle.on_body_pose_changed(runtime.on_body_pose_changed)
        context.lifecycle.on_dispose(runtime.dispose)


revo2_hand_physics_extension = Revo2HandPhysicsExtension()
