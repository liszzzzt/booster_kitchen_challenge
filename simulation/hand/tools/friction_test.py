#!/usr/bin/env python3
"""Revo2 grasp / friction test in Booster Studio's own physics (offline).

Loads a Studio scene with Studio's loader (same MuJoCo 3.10 model and options, same object
geoms, masses and friction), drives the fingers with the real revo2_hand extension code and holds the
robot like tests/offline_check.py (trunk fixed, joints on PD + gravity compensation). For each
kitchen object, alone on the kitchen table:

  1. setup     robot 0.36 m behind / 0.10 m left of the object, other free objects cleared away,
               right hand open 4 cm beside the object (side grasp: thumb up, fingers forward,
               palm towards the object), palm centre 6 cm above the object's bottom
  2. approach  0.8 s to the contact pose (palm 1 cm off the object)
  3. close     preset (power / pinch)                                    1.2 s
  4. lift      10 cm straight up (IK every 20 ms) + 0.5 s
  5. hold      2.0 s
  6. shake     right shoulder_pitch +-0.25 rad at 1 Hz                   3.0 s (+1 s settle)
  7. pull      extra downward force on the object, ramp 4 N/s, until it slips > 10 mm (max 100 N)

Logged every 10 ms (log_<object>.csv): per finger and in total the normal force Fn, friction force
Ft, worst utilisation Ft/(mu Fn) (1 = at the friction-cone limit = slipping), mu of the pair, the
net hand force on the object, object slip relative to the hand. summary.json: options, the friction
parameters of every hand / object collision geom, and one result per object.

    BOOSTER_SIM_LIBRARY_ROOT=<library> python friction_test.py --sim-src <booster_robocup_sim> \
        --scene <library>/scenes/default_pitch_T2_2.bscene --out friction_results

Needs mujoco==3.10.0 + numpy and Studio's booster_robocup_sim source (copied out of the container).
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OBJECTS = {   # object body -> grasp preset
    "coffee_cup": "power", "milk": "power", "cereal": "power", "breakfast_bowl": "power",
    "plate": "power", "kitchen_knife": "pinch", "breakfast_spoon": "pinch",
}
FINGER_KEYS = ("thumb", "index", "middle", "ring", "pinky", "palm")
G = 9.81


def load(a):
    sys.path.insert(0, a.sim_src)
    import mujoco
    from core.scene_packager import load_package
    ls = load_package(a.scene)
    return mujoco, ls.model


def load_ext(path):
    spec = importlib.util.spec_from_file_location("revo2_ext", path)
    ext = importlib.util.module_from_spec(spec)
    sys.modules["revo2_ext"] = ext
    spec.loader.exec_module(ext)
    return ext


def load_ik(path):
    spec = importlib.util.spec_from_file_location("arm_ik", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Bench:
    def __init__(self, mujoco, m, ext, ik_mod, robot="robot1", side="right"):
        self.mj, self.m, self.ext, self.ikm = mujoco, m, ext, ik_mod
        self.robot, self.side = robot, side
        self.s = 1.0 if side == "left" else -1.0
        from robot.base_robot import BoosterT2Config
        self.names = BoosterT2Config().sim_joint_names
        root = m.joint(f"{robot}_world_joint").id
        self.off, self.voff = int(m.jnt_qposadr[root]), int(m.jnt_dofadr[root])
        self.act = [m.actuator(f"{robot}_{n}").id for n in self.names]
        self.hand = m.body(f"{robot}_{side}_revo2_hand").id
        self.wrist = m.body(f"{robot}_{side}_wrist_roll_link").id
        self.trunk = m.body(f"{robot}_trunk").id
        # trunk held in place: besides resetting it every step, give its free joint a huge armature so the
        # leg PD inside one step cannot shake it (that shook the arms at ~1.5 cm/s before)
        m.dof_armature[self.voff:self.voff + 6] = 1e6
        # hand subtree bodies -> finger key
        self.body_key = {}
        for b in range(m.nbody):
            bb = b
            while bb > 0 and bb != self.hand:
                bb = int(m.body_parentid[bb])
            if bb == self.hand:
                n = m.body(b).name
                self.body_key[b] = next((k for k in FINGER_KEYS if f"_{k}_" in n), "palm")

    # ------------------------------------------------------------------ setup
    def reset(self):
        mj, m = self.mj, self.m
        self.d = d = mj.MjData(m)
        mj.mj_forward(m, d)
        lc = _Lifecycle()
        self.ext.revo2_hand_physics_extension = self.ext.Revo2HandPhysicsExtension()
        cfg = self.ext.Revo2HandConfig(ros=False)
        self.ext.revo2_hand_physics_extension.contribute(_Ctx(lc), cfg)
        self.rt = self.ext.revo2_hand_physics_extension.runtime
        self.cb = lc.cb
        for f in self.cb["on_initialize"]:
            f(m, d)
        self.q_hold = d.qpos[self.off:self.off + 7].copy()
        self.q_des = d.qpos[self.off + 7:self.off + 38].copy()
        self.obj = None
        self.t_shake = None
        self.rel0 = None

    def palm_world(self):
        d = self.d
        R = d.xmat[self.wrist].reshape(3, 3)
        p = d.xpos[self.wrist] + R @ np.array([0.0, self.s * 0.120, -0.012])
        normal = -R[:, 2]                 # palm normal (out of the palm)
        fingers = self.s * R[:, 1]
        thumb = R[:, 0]
        return p, normal, fingers, thumb

    def obj_vertices(self, body):
        """Collision-geometry points of a body, in the body frame."""
        m, mj = self.m, self.mj
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != body or (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0):
                continue
            Rg = np.zeros(9); mj.mju_quat2Mat(Rg, m.geom_quat[g]); Rg = Rg.reshape(3, 3)
            if m.geom_type[g] == mj.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            else:
                a = m.geom_aabb[g]
                s = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)
                v = a[:3] + a[3:] * s
            pts.append(m.geom_pos[g] + v @ Rg.T)
        return np.vstack(pts)

    # ------------------------------------------------------------------ table scenario
    def world_aabb(self, body):
        mj, d = self.mj, self.d
        v = self.obj_vertices(body)
        R = d.xmat[body].reshape(3, 3)
        w = d.xpos[body] + v @ R.T
        return w.min(0), w.max(0)

    def set_arm(self, q7, teleport=False):
        for n, v in zip(self.ikm.joint_names(self.side), q7):
            k = self.names.index(n)
            self.q_des[k] = v
            if teleport:
                self.d.qpos[self.off + 7 + k] = v
                self.d.qvel[self.voff + 6 + k] = 0
        self.arm_q = np.array(q7, float)
        if teleport:
            self.mj.mj_forward(self.m, self.d)
            for f in self.cb.get("on_body_pose_changed", []):
                f(self.m, self.d, self.hand)
            self.mj.mj_forward(self.m, self.d)

    def ik_world(self, p_w, R_w, q0=None, **kw):
        d = self.d
        Rt = d.xmat[self.trunk].reshape(3, 3)
        p_t = Rt.T @ (np.asarray(p_w) - d.xpos[self.trunk])
        q, info = self.ik.solve(p_t, R=Rt.T @ R_w, q0=q0, **kw)
        return q, info

    def setup_table(self, name, fwd=0.36, right=0.10, gap=0.010, grasp_height=0.06, along=0.005):
        """Robot in front of the table, only `name` left on it, hand open 4 cm beside it."""
        m, d, mj = self.m, self.d, self.mj
        self.ik = self.ikm.ArmIK(self.side, "palm")
        b = m.body(name).id
        j = m.body_jntadr[b]
        self.obj, self.obj_qadr, self.obj_dadr = b, int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j])
        self.obj_mass = float(m.body_subtreemass[b])
        # clear the table: every other free object goes to the far floor
        k = 0
        for bb in range(m.nbody):
            jj = m.body_jntadr[bb]
            if bb == b or jj < 0 or m.jnt_type[jj] != mj.mjtJoint.mjJNT_FREE or m.body(bb).name.startswith(self.robot):
                continue
            qa = m.jnt_qposadr[jj]
            d.qpos[qa:qa + 3] = [4.0 + 0.6 * k, 3.0, 0.3]; k += 1
        mj.mj_forward(m, d)
        lo, hi = self.world_aabb(b)
        ctr = (lo + hi) / 2
        # trunk yaw is -90 deg: robot forward = world -y, robot right = world -x
        self.q_hold[0] = ctr[0] + right
        self.q_hold[1] = ctr[1] + fwd
        d.qpos[self.off:self.off + 7] = self.q_hold
        mj.mj_forward(m, d)
        self.R_grasp = np.column_stack([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])   # thumb up, fingers -y, palm +x
        n = -self.R_grasp[:, 2]
        half_w = (hi[0] - lo[0]) / 2
        z = lo[2] + min(grasp_height, 0.45 * (hi[2] - lo[2]))
        self.p_contact = np.array([ctr[0], ctr[1] + along, z]) - n * (half_w + gap)
        self.obj_size = dict(x=float(hi[0] - lo[0]), y=float(hi[1] - lo[1]), z=float(hi[2] - lo[2]))
        q_pre, info = self.ik_world(self.p_contact - n * 0.04, self.R_grasp)
        assert info["ok"], ("pre-grasp IK", info)
        self.set_arm(q_pre, teleport=True)
        self.q_contact, info = self.ik_world(self.p_contact, self.R_grasp, q0=q_pre)
        assert info["ok"], ("contact IK", info)

    def move_arm(self, q_to, T, log=None, phase=""):
        q_from = self.arm_q.copy()
        n = int(round(T / 0.02))
        for i in range(1, n + 1):
            s = i / n; s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            self.set_arm(q_from + s * (q_to - q_from))
            self.step(0.02, log, phase)

    def lift(self, dz, T, log=None, phase="lift"):
        p0 = self.p_contact.copy()
        q = self.arm_q.copy()
        n = int(round(T / 0.02))
        for i in range(1, n + 1):
            s = i / n; s = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
            q, info = self.ik_world(p0 + np.array([0, 0, dz * s]), self.R_grasp, q0=q, restarts=0, iters=60)
            self.set_arm(q)
            self.step(0.02, log, phase)

    # ------------------------------------------------------------------ stepping
    def _pair(self, c):
        """(finger key, object-is-geom1) if contact is hand<->object, else None."""
        b1, b2 = int(self.m.geom_bodyid[c.geom1]), int(self.m.geom_bodyid[c.geom2])
        if self.obj is None:
            return None
        if b1 == self.obj and b2 in self.body_key:
            return self.body_key[b2]
        if b2 == self.obj and b1 in self.body_key:
            return self.body_key[b1]
        return None

    def step(self, T, log=None, phase="", ext_force=None):
        m, d, mj = self.m, self.d, self.mj
        nsteps = int(round(T / m.opt.timestep))
        kp, kd = 150.0, 4.0
        for i in range(nsteps):
            if self.t_shake is not None:
                k = self.names.index(f"{self.side}_shoulder_pitch_joint")
                self.q_des[k] = self.arm_q[0] + 0.25 * np.sin(2 * np.pi * 1.0 * (d.time - self.t_shake))
            d.qpos[self.off:self.off + 7] = self.q_hold
            d.qvel[self.voff:self.voff + 6] = 0
            if self.obj is not None:
                d.xfrc_applied[self.obj] = 0
                if ext_force is not None:
                    d.xfrc_applied[self.obj, 2] = -ext_force(d.time)
            for f in self.cb["on_before_step"]:
                f(None)
            q = d.qpos[self.off + 7:self.off + 38]; v = d.qvel[self.voff + 6:self.voff + 37]
            tau = kp * (self.q_des - q) - kd * v + d.qfrc_bias[self.voff + 6:self.voff + 37]
            d.ctrl[self.act] = np.clip(tau, m.actuator_ctrlrange[self.act, 0], m.actuator_ctrlrange[self.act, 1])
            mj.mj_step(m, d)
            for f in self.cb["on_post_step"]:
                f(None)
            if log is not None and i % 5 == 4:
                row = self.measure(phase, ext_force(d.time) if ext_force else 0.0)
                log.append(row)
                if phase == "pull" and (row["slip_mm"] > 10.0 or row["n_contacts"] == 0):
                    return row
        return None

    def hand_frame_obj(self):
        d = self.d
        R = d.xmat[self.hand].reshape(3, 3)
        return R.T @ (d.xpos[self.obj] - d.xpos[self.hand])

    def measure(self, phase, f_ext):
        m, d, mj = self.m, self.d, self.mj
        per = {k: dict(fn=0.0, ft=0.0, n=0, util=0.0) for k in FINGER_KEYS}
        f6 = np.zeros(6)
        fsum = np.zeros(3)
        utils, mus = [], []
        for i in range(d.ncon):
            c = d.contact[i]
            key = self._pair(c)
            if key is None:
                continue
            mj.mj_contactForce(m, d, i, f6)
            fn = f6[0]
            ft = float(np.hypot(f6[1], f6[2]))
            mu = float(c.friction[0])
            u = ft / (mu * fn) if fn > 1e-6 else 0.0
            p = per[key]; p["fn"] += fn; p["ft"] += ft; p["n"] += 1; p["util"] = max(p["util"], u)
            utils.append(u); mus.append(mu)
            # contact force on the object, world frame (frame rows = normal, t1, t2; normal points geom1->geom2)
            fr = c.frame.reshape(3, 3)
            fw = fr.T @ f6[:3]
            obj_is_g2 = int(m.geom_bodyid[c.geom2]) == self.obj
            fsum += fw if obj_is_g2 else -fw
        rel = self.hand_frame_obj()
        slip = 1000 * float(np.linalg.norm(rel - self.rel0)) if getattr(self, "rel0", None) is not None else 0.0
        row = dict(t=round(float(d.time), 3), phase=phase, f_ext=round(f_ext, 3),
                   n_contacts=len(utils), grip_fn=round(sum(p["fn"] for p in per.values()), 3),
                   friction_ft=round(sum(p["ft"] for p in per.values()), 3),
                   max_util=round(max(utils) if utils else 0.0, 3),
                   mu=round(float(np.median(mus)), 3) if mus else 0.0,
                   contact_force_on_obj_z=round(float(fsum[2]), 3),
                   slip_mm=round(slip, 3),
                   obj_speed_rel=round(float(np.linalg.norm(d.qvel[self.obj_dadr:self.obj_dadr + 3] -
                                                            d.cvel[self.hand][3:])), 4))
        for k, p in per.items():
            row[f"{k}_fn"] = round(p["fn"], 3); row[f"{k}_ft"] = round(p["ft"], 3)
            row[f"{k}_util"] = round(p["util"], 3); row[f"{k}_n"] = p["n"]
        return row


class _Lifecycle:
    def __init__(self):
        self.cb = {}

    def __getattr__(self, k):
        if k.startswith("on_"):
            return lambda f: self.cb.setdefault(k, []).append(f)
        raise AttributeError(k)


class _Ctx:
    def __init__(self, lc):
        self.lifecycle = lc



def run_object_table(bench, name, preset, lift=0.10, **grasp):
    bench.reset()
    bench.setup_table(name, **grasp)
    bench.rt.submit({"side": bench.side, "preset": "open"})
    bench.step(0.6)                                    # object settles on the table
    z_table = float(bench.d.xpos[bench.obj][2])
    log = []
    bench.move_arm(bench.q_contact, 0.8, log, "approach")
    bench.rt.submit({"side": bench.side, "preset": preset})
    bench.step(1.2, log, "close")
    bench.lift(lift, 1.5, log, "lift")
    bench.step(0.5, log, "lift")
    lifted = float(bench.d.xpos[bench.obj][2]) - z_table
    bench.rel0 = bench.hand_frame_obj()
    bench.step(2.0, log, "hold")
    hold_rows = [r for r in log if r["phase"] == "hold"]
    bench.t_shake = bench.d.time
    bench.step(3.0, log, "shake")
    bench.t_shake = None
    k = bench.names.index(f"{bench.side}_shoulder_pitch_joint"); bench.q_des[k] = bench.arm_q[0]
    bench.step(1.0, log, "settle")
    shake_rows = [r for r in log if r["phase"] in ("shake", "settle")]
    held = lambda r: r["n_contacts"] > 0 and r["slip_mm"] < 30
    dropped_before_pull = not held(log[-1]) or lifted < 0.5 * lift
    f_slip = None
    if not dropped_before_pull:
        bench.rel0 = bench.hand_frame_obj()
        t0 = bench.d.time
        end = bench.step(25.0, log, "pull", ext_force=lambda t: 4.0 * (t - t0))
        f_slip = end["f_ext"] if end else 100.0
    w = bench.obj_mass * G
    h = hold_rows[-1]
    summ = dict(
        object=name, preset=preset, mass_kg=round(bench.obj_mass, 3), weight_N=round(w, 2),
        size_m={k: round(v, 3) for k, v in bench.obj_size.items()}, mu=h["mu"],
        lifted_cm=round(100 * lifted, 1),
        hold_ok=bool(held(h) and lifted > 0.5 * lift and h["slip_mm"] < 5.0),
        slip_hold_mm=h["slip_mm"], slip_shake_mm=shake_rows[-1]["slip_mm"],
        grip_fn_hold_N=h["grip_fn"], friction_ft_hold_N=h["friction_ft"],
        contact_force_on_obj_z_hold_N=h["contact_force_on_obj_z"],
        max_util_hold=max(r["max_util"] for r in hold_rows[-50:]),
        max_util_shake=max(r["max_util"] for r in shake_rows),
        extra_load_at_slip_N=None if f_slip is None else round(f_slip, 2),
        max_mass_kg=None if f_slip is None else round((w + f_slip) / G, 3),
        safety_factor=None if f_slip is None else round((w + f_slip) / w, 2),
        per_finger_hold={k: dict(fn=h[f"{k}_fn"], ft=h[f"{k}_ft"], util=h[f"{k}_util"], contacts=h[f"{k}_n"])
                         for k in FINGER_KEYS},
        closure_hold=next(x["closure"] for x in bench.rt.state() if x["side"] == bench.side),
    )
    return summ, log


def friction_table(mujoco, m, robot="robot1", side="right"):
    rows = []
    for g in range(m.ngeom):
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue
        bn = m.body(int(m.geom_bodyid[g])).name
        if not (bn.startswith(f"{robot}_{side}_") and any(k in bn for k in ("revo2", "thumb", "index", "middle", "ring", "pinky"))) \
                and bn not in OBJECTS:
            continue
        rows.append(dict(body=bn, geom=m.geom(g).name or f"#{g}", slide=float(m.geom_friction[g][0]),
                         torsion=float(m.geom_friction[g][1]), roll=float(m.geom_friction[g][2]),
                         condim=int(m.geom_condim[g]), priority=int(m.geom_priority[g]),
                         solref=[float(x) for x in m.geom_solref[g]], solimp=[float(x) for x in m.geom_solimp[g][:3]]))
    o = m.opt
    opt = dict(timestep=o.timestep, cone=["pyramidal", "elliptic"][int(o.cone)], impratio=o.impratio,
               solver=int(o.solver), iterations=int(o.iterations), noslip_iterations=int(o.noslip_iterations),
               integrator=int(o.integrator))
    return rows, opt


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim-src", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--extension", default=os.path.join(HERE, "..", "plugin", "revo2_hand", "extension.py"))
    ap.add_argument("--arm-ik", default=os.path.join(HERE, "..", "..", "arm", "arm_ik.py"))
    ap.add_argument("--objects", default=",".join(OBJECTS))
    ap.add_argument("--out", default="friction_results")
    ap.add_argument("--preset", default=None, help="override the per-object preset (power / pinch / fist ...)")
    ap.add_argument("--gap", type=float, default=0.010, help="palm-to-object gap at the contact pose [m]")
    ap.add_argument("--grasp-height", type=float, default=0.06, help="palm centre above the object bottom [m]")
    ap.add_argument("--along", type=float, default=0.005,
                    help="hand pulled back along the fingers by this much (+ = object nearer the fingertips) [m]")
    a = ap.parse_args()
    mujoco, m = load(a)
    ext = load_ext(a.extension)
    ikm = load_ik(a.arm_ik)
    os.makedirs(a.out, exist_ok=True)
    table, opt = friction_table(mujoco, m)
    bench = Bench(mujoco, m, ext, ikm)
    results = []
    for name in a.objects.split(","):
        preset = a.preset or OBJECTS.get(name, "power")
        try:
            summ, log = run_object_table(bench, name, preset, gap=a.gap, grasp_height=a.grasp_height, along=a.along)
        except AssertionError as exc:          # the arm cannot reach the grasp pose
            print(json.dumps({"object": name, "error": f"IK: {exc}"}, ensure_ascii=False))
            results.append({"object": name, "preset": preset, "error": f"arm IK cannot reach the grasp pose: {exc}"})
            continue
        results.append(summ)
        with open(os.path.join(a.out, f"log_{name}.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(log[0].keys()))
            w.writeheader(); w.writerows(log)
        print(json.dumps({k: summ[k] for k in ("object", "mass_kg", "lifted_cm", "hold_ok", "slip_hold_mm", "slip_shake_mm",
                                                "grip_fn_hold_N", "max_util_hold", "extra_load_at_slip_N",
                                                "safety_factor")}, ensure_ascii=False))
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(dict(options=opt, grasp=dict(gap=a.gap, grasp_height=a.grasp_height, along=a.along),
                       friction_params=table, results=results), f, indent=2, ensure_ascii=False, default=float)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
