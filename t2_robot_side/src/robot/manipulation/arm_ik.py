#!/usr/bin/env python3
"""T2 arm forward / inverse kinematics (numpy only, no MuJoCo, no SDK).

Frame: everything is expressed in the **trunk frame** (body `trunk` of the T2 model):
    x forward, y left, z up, origin at the trunk body origin.
In T2 both arms hang directly off `trunk` (the waist is *below* it), so the hand pose in the
trunk frame depends only on the 7 arm joints — waist / legs / walking do not change it.

Kinematic parameters are copied from Booster Studio's builtin T2 (T2_v1p2_31dof.xml); the arm
chain is identical in booster_assets T2_31dof_Revo2Hand.xml. Hand points are given in the
`*_wrist_roll_link` frame, taken from the Revo2 hand model:
    right hand: fingers -y, palm normal -z, thumb +x      left hand: fingers +y, palm normal -z, thumb +x
At all-zero joints the arms point straight out sideways (T-pose).

    from arm_ik import ArmIK, orientation
    ik = ArmIK("right")                                   # point="palm" (default) | "grasp" | "wrist"
    q, info = ik.solve([0.35, -0.20, 0.05], R=orientation("right", "side"), q0=current_q7)
    if info["ok"]: ...                                     # q = 7 joint angles in ARM_JOINTS order
    p, R = ik.fk(q)
"""
from __future__ import annotations

import numpy as np

ARM_JOINTS = ["shoulder_pitch", "shoulder_roll", "elbow_pitch", "elbow_yaw",
              "wrist_pitch", "wrist_yaw", "wrist_roll"]


def joint_names(side: str) -> list[str]:
    """Full joint names, e.g. right_shoulder_pitch_joint ... in IK order."""
    return [f"{side}_{j}_joint" for j in ARM_JOINTS]


# ---------------------------------------------------------------- model (from T2_v1p2_31dof.xml)
def _chain(side: str):
    s = 1.0 if side == "left" else -1.0
    qx = 0.0697583
    # (body pos in parent, body quat wxyz in parent, joint axis)
    return [
        ((0.0615, s * 0.11422, 0.16458), (0.997564, s * qx, 0, 0), (0, 1, 0)),   # shoulder_pitch
        ((0.0, s * 0.04926, -0.0107), (0.997564, -s * qx, 0, 0), (1, 0, 0)),     # shoulder_roll
        ((0.00662, s * 0.1084, 0.0), (1, 0, 0, 0), (0, 1, 0)),                   # elbow_pitch
        ((0.0059, s * 0.109, 0.0), (1, 0, 0, 0), (0, 0, 1)),                     # elbow_yaw
        ((-0.00092228, s * 0.094, 0.0), (1, 0, 0, 0), (0, 1, 0)),                # wrist_pitch
        ((0.0, s * 0.118, 0.0), (1, 0, 0, 0), (0, 0, 1)),                        # wrist_yaw
        ((0.0, 0.0, 0.0), (1, 0, 0, 0), (1, 0, 0)),                              # wrist_roll
    ]


LIMITS = {
    "left": np.array([[-3.01942, 1.71042], [-1.51844, 1.51844], [-2.05949, 2.05949], [-2.10312, 0.0],
                      [-2.05949, 2.05949], [-0.748746, 0.678933], [-1.36136, 0.925025]]),
    "right": np.array([[-3.01942, 1.71042], [-1.51844, 1.51844], [-2.05949, 2.05949], [0.0, 2.10312],
                       [-2.05949, 2.05949], [-0.678933, 0.748746], [-0.925025, 1.36136]]),
}

# points on the hand, in the *_wrist_roll_link frame (same as the sites in the Revo2 models)
HAND_POINTS = {
    "wrist": (0.0, 0.0, 0.0),
    "palm": (0.0, 0.120, -0.012),    # palm centre (left: +y, right: -y)
    "grasp": (0.0, 0.130, -0.050),   # ~4 cm in front of the palm: where a held object's centre sits
}


def _quat2mat(q):
    w, x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _axis_angle(axis, a):
    k = np.asarray(axis, float)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def rot_error(R_cur, R_tgt):
    """Rotation vector (trunk frame) taking R_cur to R_tgt."""
    Re = R_tgt @ R_cur.T
    c = np.clip((np.trace(Re) - 1) / 2, -1.0, 1.0)
    a = np.arccos(c)
    v = np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    if a < 1e-6:
        return 0.5 * v
    if np.pi - a < 1e-4:                       # 180 deg: pick the axis from the symmetric part
        B = (Re + np.eye(3)) / 2
        k = B[:, int(np.argmax(np.diag(B)))]
        return np.pi * k / np.linalg.norm(k)
    return a / (2 * np.sin(a)) * v


# ---------------------------------------------------------------- orientations
def orientation(side: str, kind: str = "side") -> np.ndarray:
    """Common hand orientations (rotation of *_wrist_roll_link in the trunk frame).

    side  : fingers forward, palm facing inwards (towards the body midline), thumb up
            -> side grasp of a cup / bottle standing on a table
    down  : fingers forward, palm down, thumb inwards -> top grasp
    front : fingers up, palm facing forward -> pushing / "stop" pose
    """
    s = 1.0 if side == "left" else -1.0
    fingers = {"side": (1, 0, 0), "down": (1, 0, 0), "front": (0, 0, 1)}[kind]
    palm = {"side": (0, -s, 0), "down": (0, 0, -1), "front": (1, 0, 0)}[kind]
    # hand frame: +y = s * fingers, -z = palm normal, x = y cross z
    y = s * np.asarray(fingers, float)
    z = -np.asarray(palm, float)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def rpy_matrix(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll)."""
    return _axis_angle((0, 0, 1), yaw) @ _axis_angle((0, 1, 0), pitch) @ _axis_angle((1, 0, 0), roll)


# ---------------------------------------------------------------- IK
class ArmIK:
    def __init__(self, side: str = "right", point: str | tuple = "palm"):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        self.side = side
        self.names = joint_names(side)
        self.lim = LIMITS[side].copy()
        s = 1.0 if side == "left" else -1.0
        self._links = [(np.array(p, float), _quat2mat(q), np.array(a, float)) for p, q, a in _chain(side)]
        if isinstance(point, str):
            x, y, z = HAND_POINTS[point]
            point = (x, s * y, z)
        self.point = np.array(point, float)

    # forward kinematics -------------------------------------------------
    def fk_all(self, q):
        """Joint origins/axes (trunk frame) and the tool frame (pos, R)."""
        R = np.eye(3)
        p = np.zeros(3)
        origins, axes = [], []
        for (pos, Rb, ax), qi in zip(self._links, q):
            p = p + R @ pos
            R = R @ Rb
            origins.append(p.copy())
            axes.append(R @ ax)
            R = R @ _axis_angle(ax, qi)
        tool = p + R @ self.point
        return np.array(origins), np.array(axes), tool, R

    def fk(self, q):
        """(position of the hand point, rotation of *_wrist_roll_link) in the trunk frame."""
        _, _, p, R = self.fk_all(np.asarray(q, float))
        return p, R

    def jacobian(self, q):
        o, a, p, R = self.fk_all(np.asarray(q, float))
        Jv = np.cross(a, p - o).T          # 3x7
        Jw = a.T                           # 3x7
        return Jv, Jw, p, R

    # inverse kinematics -------------------------------------------------
    def solve(self, pos, R=None, q0=None, *, axis=None, rot_weight=0.3, pos_tol=0.003,
              rot_tol=0.03, iters=200, restarts=12, posture_gain=0.05, seed=0):
        """Damped-least-squares IK.

        pos   : target position of the hand point, trunk frame [m]
        R     : optional 3x3 target rotation of the hand (see orientation())
        axis  : optional (hand_axis, direction) — only align one hand axis instead of the full R,
                e.g. axis=((0, 0, -1), (0, 0, -1)) = "palm faces down, any yaw"
        q0    : start / preferred posture (normally the current joint angles)
        Returns (q, info); info = {ok, pos_err, rot_err, iters}. q is always within joint limits.
        """
        pos = np.asarray(pos, float)
        lo, hi = self.lim[:, 0] + 1e-3, self.lim[:, 1] - 1e-3
        q_pref = np.clip(np.zeros(7) if q0 is None else np.asarray(q0, float), lo, hi)
        rng = np.random.default_rng(seed)
        seeds = [q_pref.copy()] + [rng.uniform(lo, hi) for _ in range(restarts)]
        best = None
        for k, q in enumerate(seeds):
            q, info = self._solve_from(q, pos, R, axis, q_pref, rot_weight, pos_tol, rot_tol,
                                       iters, posture_gain, lo, hi)
            info["seed"] = k
            cost = info["pos_err"] + 0.1 * info["rot_err"] + (0 if info["ok"] else 10) \
                + 0.01 * np.linalg.norm(q - q_pref)
            if best is None or cost < best[0]:
                best = (cost, q, info)
            if info["ok"] and k == 0:      # converged from the current posture: keep it
                break
        return best[1], best[2]

    def _task(self, q, pos, R, axis, rot_weight):
        Jv, Jw, p, Rc = self.jacobian(q)
        e = [pos - p]
        J = [Jv]
        rerr = 0.0
        if R is not None:
            er = rot_error(Rc, np.asarray(R, float))
            rerr = float(np.linalg.norm(er))
            e.append(rot_weight * er)
            J.append(rot_weight * Jw)
        elif axis is not None:
            a_cur = Rc @ np.asarray(axis[0], float)
            a_tgt = np.asarray(axis[1], float) / np.linalg.norm(axis[1])
            er = np.cross(a_cur, a_tgt)
            rerr = float(np.arccos(np.clip(a_cur @ a_tgt, -1, 1)))
            P = np.eye(3) - np.outer(a_cur, a_cur)      # rotation about the axis itself is free
            e.append(rot_weight * er)
            J.append(rot_weight * P @ Jw)
        return np.concatenate(e), np.vstack(J), float(np.linalg.norm(pos - p)), rerr

    def _solve_from(self, q, pos, R, axis, q_pref, rot_weight, pos_tol, rot_tol, iters, kpost, lo, hi):
        has_rot = R is not None or axis is not None
        for it in range(iters):
            e, J, perr, rerr = self._task(q, pos, R, axis, rot_weight)
            if perr < pos_tol and (not has_rot or rerr < rot_tol):
                return q, dict(ok=True, pos_err=perr, rot_err=rerr, iters=it)
            lam = 1e-4 + 0.05 * float(e @ e)              # more damping far from the goal
            JJt = J @ J.T + lam * np.eye(len(e))
            dq = J.T @ np.linalg.solve(JJt, e)
            N = np.eye(7) - J.T @ np.linalg.solve(JJt, J)  # null space: drift toward q_pref
            dq += N @ (kpost * (q_pref - q))
            n = np.max(np.abs(dq))
            if n > 0.25:
                dq *= 0.25 / n
            q = np.clip(q + dq, lo, hi)
        e, J, perr, rerr = self._task(q, pos, R, axis, rot_weight)
        ok = perr < pos_tol and (not has_rot or rerr < rot_tol)
        return q, dict(ok=ok, pos_err=perr, rot_err=rerr, iters=iters)


if __name__ == "__main__":                 # quick self-test: python arm_ik.py
    np.set_printoptions(precision=3, suppress=True)
    for side in ("right", "left"):
        ik = ArmIK(side)
        p0, _ = ik.fk(np.zeros(7))
        print(f"{side}: palm at zero pose (T-pose) = {p0}")
        y = -0.20 if side == "right" else 0.20
        for kind in ("side", "down"):
            q, info = ik.solve([0.35, y, -0.05], R=orientation(side, kind))
            print(f"  target (0.35,{y:+.2f},-0.05) {kind:5s}: ok={info['ok']} "
                  f"pos_err={info['pos_err']*1000:.1f}mm rot_err={np.degrees(info['rot_err']):.1f}deg q={q}")
