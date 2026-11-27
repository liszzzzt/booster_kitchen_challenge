#!/usr/bin/env python3
"""Offline fingertip friction test in Booster Studio's own physics (force-controlled, clean).

Same setup as t2_robot_side/src/robot/manipulation/finger_drag.py (one finger straight and pointing down at the kitchen table,
tilted so the table pushes it into extension), but instead of dragging the arm, the arm holds still and a
horizontal force ramping 1 N/s is applied to the fingertip link, sideways to the finger's bending plane.
Slip = the fingertip's material point at the contact moves tangentially > 5 mm/s for 5 samples (50 ms).
At that moment ft/fn of the finger's contacts is the effective friction coefficient of the simulation.

    BOOSTER_SIM_LIBRARY_ROOT=<library> python finger_friction_test.py --sim-src <booster_robocup_sim> \\
        --scene <library>/scenes/default_pitch_T2_2.bscene [--fingers index,middle] [--cone elliptic]

--cone elliptic overrides the scene's friction cone (pyramidal in the Studio kitchen scene) to compare.
Needs mujoco==3.10.0, numpy, friction_test.py (same folder), t2_robot_side/src/robot/manipulation/arm_ik.py, the plugin.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import friction_test as ft  # noqa: E402

TIPS = {"thumb": (0.1115, -0.1094, -0.0111), "index": (0.0332, -0.2158, -0.0198),
        "middle": (0.0117, -0.2295, -0.0196), "ring": (-0.0116, -0.2256, -0.0187),
        "pinky": (-0.0325, -0.2079, -0.0176)}   # right hand, *_wrist_roll_link frame


def hand_shape(finger):
    c = dict(thumb_rotation=0.0, thumb_flex=0.0, index=1.0, middle=1.0, ring=1.0, pinky=1.0)
    if finger != "thumb":
        c[finger] = 0.0
    return c


def pose_R(finger, tilt, yaw):
    a = np.array((1.0, 0, 0)) if finger == "thumb" else np.array((0, -1.0, 0))
    z = np.array((0, 0, 1.0))
    u = np.array((np.cos(yaw), np.sin(yaw), 0)); up = np.array((0, 0, 1.0))
    F = np.sin(tilt) * u - np.cos(tilt) * up
    N = -np.cos(tilt) * u - np.sin(tilt) * up
    H = np.column_stack([a, z, np.cross(a, z)]); W = np.column_stack([F, -N, np.cross(F, -N)])
    return W @ H.T


def run_finger(b, mj, m, ikm, finger, depth, tilt=0.6, rate=1.0, fmax=40.0):
    b.reset(); b.setup_table("coffee_cup")               # robot at the table; the cup is moved away
    qa = b.obj_qadr; b.d.qpos[qa:qa + 3] = [5, 5, 0.3]; mj.mj_forward(m, b.d); b.obj = None
    d = b.d
    ik = ikm.ArmIK("right", point=TIPS[finger])
    b.rt.submit({"side": "right", "closure": hand_shape(finger)})
    p = np.array([0.30, -0.18, -0.14]); R = None
    for yaw in (0.5, 1.0, 0.0, -0.5, 1.57, -1.0):
        Rc = pose_R(finger, tilt, yaw)
        if all(ik.solve(p + np.array([0, 0, dz]), R=Rc)[1]["ok"] for dz in (0.0, -0.06, -0.10)):
            R = Rc; break
    if R is None:
        return dict(finger=finger, depth_mm=depth * 1000, error="IK: fingertip pose not reachable")
    dv = R[:, 2] if finger == "thumb" else R[:, 0]
    dv = np.array([dv[0], dv[1], 0.0]); dv /= np.linalg.norm(dv)
    dw = d.xmat[b.trunk].reshape(3, 3) @ dv
    q, _ = ik.solve(p, R=R); b.set_arm(q, teleport=True); b.step(1.2)
    tipbody = m.body(f"robot1_right_{finger}_distal_link").id
    fbodies = {bb for bb, k in b.rt.hands[("robot1", "right")].body_key.items() if k == finger}
    z = p[2]
    while True:
        z -= 0.002
        if z < -0.35:
            return dict(finger=finger, depth_mm=depth * 1000, error="no contact")
        q, _ = ik.solve([p[0], p[1], z], R=R, q0=q, restarts=0); b.move_arm(q, 0.12)
        if [x for x in b.rt.contacts() if x["side"] == "right"][0]["fingers"][finger]["fn"] > 0.3:
            break
    q, _ = ik.solve([p[0], p[1], z - depth], R=R, q0=q, restarts=0); b.move_arm(q, 0.5); b.step(0.8)
    jp = np.zeros((3, m.nv)); t0 = d.time; rows = []
    while d.time - t0 < fmax / rate:
        F = rate * (d.time - t0)
        d.xfrc_applied[tipbody] = 0; d.xfrc_applied[tipbody, :3] = F * dw
        b.step(0.01)
        c = [x for x in b.rt.contacts() if x["side"] == "right"][0]["fingers"][finger]
        slip = 0.0
        for i in range(d.ncon):
            cc = d.contact[i]
            b1, b2 = int(m.geom_bodyid[cc.geom1]), int(m.geom_bodyid[cc.geom2])
            fb_ = b1 if b1 in fbodies else (b2 if b2 in fbodies else None)
            if fb_ is None:
                continue
            mj.mj_jac(m, d, jp, None, cc.pos, fb_); v = jp @ d.qvel; nrm = cc.frame[:3]
            slip = max(slip, float(np.linalg.norm(v - np.dot(v, nrm) * nrm)))
        rows.append((F, c["fn"], c["ft"], slip))
        if len(rows) > 5 and all(r[3] > 0.005 for r in rows[-5:]):
            F, fn, ftv, _ = rows[-5]
            return dict(finger=finger, depth_mm=depth * 1000, slip_at_force_N=round(F, 2), fn_N=round(fn, 2),
                        ft_N=round(ftv, 2), mu_eff=round(ftv / fn, 3) if fn > 0 else None)
    F, fn, ftv, _ = rows[-1]
    return dict(finger=finger, depth_mm=depth * 1000, slip_at_force_N=None, fn_N=round(fn, 2), ft_N=round(ftv, 2),
                mu_eff_lower_bound=round(ftv / fn, 3) if fn > 0 else None, note=f"no slip up to {fmax} N")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim-src", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--extension", default=os.path.join(HERE, "..", "plugin", "revo2_hand", "extension.py"))
    ap.add_argument("--arm-ik", default=os.path.join(HERE, "..", "..", "..", "t2_robot_side", "src", "robot", "manipulation", "arm_ik.py"))
    ap.add_argument("--fingers", default="index,middle,ring,pinky,thumb")
    ap.add_argument("--depths", default="2,4", help="press depth after first contact [mm]")
    ap.add_argument("--cone", choices=["scene", "elliptic", "pyramidal"], default="scene")
    ap.add_argument("--out", default="finger_friction_offline.json")
    a = ap.parse_args()
    mj, m = ft.load(a)
    if a.cone != "scene":
        m.opt.cone = 1 if a.cone == "elliptic" else 0
    ext = ft.load_ext(a.extension); ikm = ft.load_ik(a.arm_ik)
    b = ft.Bench(mj, m, ext, ikm)
    out = []
    for f in a.fingers.split(","):
        for dep in a.depths.split(","):
            r = run_finger(b, mj, m, ikm, f, float(dep) / 1000)
            r["cone"] = ["pyramidal", "elliptic"][int(m.opt.cone)]
            print(json.dumps(r, ensure_ascii=False), flush=True)
            out.append(r)
    with open(a.out, "w") as fh:
        json.dump(out, fh, indent=2)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
