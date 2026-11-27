#!/usr/bin/env python3
"""Solve the head-camera extrinsics from collect.py's samples (no ROS needed).

    python3 solve.py [--out output] [--drop 3,7]

Prints and writes <out>/handeye.json:
    T_mount_cam   camera optical frame (x right, y down, z forward) in the mount link: xyz, rpy, quaternion,
                  plus a URDF <joint> to paste
    T_ref_board   where the board was (sanity check: compare with where you put it)
    per-sample reprojection error; samples far above the rest are listed as outliers (--drop them)
and compares the result with the Studio model and the URDF (t2_description) camera poses.

Studio's camera_info puts the principal point at (w/2, h/2); OpenCV's pixel convention needs
(w/2 - 0.5, h/2 - 0.5). Samples collected with --tf sim get this shift (2 mm better in tests); --no-shift
turns it off.
"""
import argparse
import json
import os

import numpy as np

import board
import handeye as he

HERE = os.path.dirname(os.path.abspath(__file__))


def fmt(T):
    d = he.describe(T)
    return (f"xyz [{', '.join(f'{v:.4f}' for v in d['xyz'])}] m   "
            f"rpy [{', '.join(f'{v:.4f}' for v in d['rpy'])}] rad   "
            f"({', '.join(f'{np.degrees(v):.2f}' for v in d['rpy'])} deg)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.join(HERE, "output"))
    ap.add_argument("--drop", default="", help="comma separated sample numbers (1-based) to leave out")
    ap.add_argument("--keep-all", action="store_true", help="no automatic outlier rejection")
    ap.add_argument("--no-shift", action="store_true", help="do not shift the Studio principal point")
    a = ap.parse_args()

    s = np.load(os.path.join(a.out, "samples.npz"))
    G, corners, K, D = list(s["G"]), list(s["corners"]), s["K"].copy(), s["D"]
    mount, ref, tf = str(s["mount"]), str(s["ref"]), str(s["tf"])
    board_name = str(s["board"]) if "board" in s.files else "stand"
    drop = {int(x) - 1 for x in a.drop.split(",") if x.strip()}
    keep = [i for i in range(len(G)) if i not in drop]
    G, corners = [G[i] for i in keep], [corners[i] for i in keep]
    if len(G) < 3:
        raise SystemExit(f"{len(G)} samples: need at least 3 (15+ recommended)")
    if tf == "sim" and not a.no_shift:
        K[0, 2] -= 0.5
        K[1, 2] -= 0.5
    spread = he.motion_spread_deg(G)
    print(f"{len(G)} samples ({mount} <- {ref}, board {board_name}), motion spread {spread:.1f} deg, K fx {K[0, 0]:.1f} "
          f"cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}")
    if spread < 10:
        print("WARNING: the head/arm hardly moved between samples; the result is poorly determined")

    object_points = board.BOARDS[board_name].object_points(float(s["square"]))
    r = he.calibrate(G, corners, object_points, K, D, reject=not a.keep_all)
    X, Y, rms = r["X"], r["Y"], r["rms"]
    dropped = [keep[i] + 1 for i in range(len(G)) if i not in r["used"]]
    keep, G, corners = [keep[i] for i in r["used"]], [G[i] for i in r["used"]], [corners[i] for i in r["used"]]
    # the board must stay fixed in the ref frame; per-sample board poses show whether it did
    boards = [he.inv(g) @ X @ he.pnp(uv, object_points, K, D)[0] for g, uv in zip(G, corners)]
    board_mm = max(np.linalg.norm(b[:3, 3] - Y[:3, 3]) for b in boards) * 1000
    board_deg = max(he.angle_deg(Y[:3, :3].T @ b[:3, :3]) for b in boards)
    outliers = [keep[i] + 1 for i, e in enumerate(rms) if e > max(3 * np.median(rms), 1.0)]

    print(f"\nreprojection RMS: closed form {np.sqrt(np.mean(r['rms0'] ** 2)):.3f} px -> "
          f"refined {np.sqrt(np.mean(rms ** 2)):.3f} px (worst {rms.max():.3f} px, sample {keep[int(rms.argmax())] + 1})")
    print(f"board pose in {ref} over the samples: max deviation {board_mm:.1f} mm, {board_deg:.2f} deg")
    if board_mm > 10 or board_deg > 1:
        print(f"WARNING: the board did not stay fixed relative to {ref} (the robot drifted, or the board moved).\n"
              f"         Collect again with a world-fixed ref, e.g. python3 collect.py --ref odom --out output_odom")
    if dropped:
        print(f"dropped automatically (far off the rest, e.g. misnumbered corners): samples {dropped}")
    if outliers:
        print(f"outliers: {outliers}  -> check those images, re-run with --drop {','.join(map(str, outliers))}")
    print(f"\nT_{mount}_camera (optical):  {fmt(X)}")
    print(f"T_{ref}_board:             {fmt(Y)}")

    comparisons = {}
    for name in he.REFERENCES:
        E = he.inv(he.reference(name)) @ X
        comparisons[name] = {"pose": he.describe(he.reference(name)),
                             "diff_mm": float(np.linalg.norm(E[:3, 3]) * 1000),
                             "diff_deg": float(he.angle_deg(E[:3, :3]))}
        print(f"vs {name:7s} {fmt(he.reference(name))}   diff {comparisons[name]['diff_mm']:.1f} mm, "
              f"{comparisons[name]['diff_deg']:.2f} deg")

    xyz, rpy = X[:3, 3], he.R_to_rpy(X[:3, :3])
    urdf = (f'<joint name="head_camera_optical_joint" type="fixed">\n'
            f'  <origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>\n'
            f'  <parent link="{mount}"/>\n  <child link="head_color_optical_frame"/>\n</joint>')
    print("\n" + urdf)

    result = {"mount": mount, "ref": ref, "samples_used": [i + 1 for i in keep], "motion_spread_deg": spread,
              "rms_px": float(np.sqrt(np.mean(rms ** 2))), "rms_px_per_sample": rms.tolist(), "outliers": outliers, "dropped": dropped,
              "board_deviation_mm": board_mm, "board_deviation_deg": board_deg,
              "T_mount_cam": he.describe(X), "T_ref_board": he.describe(Y), "compare": comparisons,
              "K_used": K.tolist(), "urdf_joint": urdf}
    path = os.path.join(a.out, "handeye.json")
    with open(path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"\nwrote {path}")


if __name__ == "__main__":
    main()
