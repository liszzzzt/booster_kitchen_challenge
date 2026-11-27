"""Head-camera extrinsics from checkerboard views. Pure numpy/OpenCV, no ROS.

Unknowns (both constant):
    X = T_mount_cam    camera optical frame in the mount link (head_pitch_link)   <- the result
    Y = T_ref_board    board in the ref link
Per sample i:
    G_i = T_mount_ref  from TF (forward kinematics)
    C_i = T_cam_board  from PnP on the detected corners
Both paths give the board in the mount frame:   X C_i = G_i Y
    static board on a table, head moves:  ref = trunk (the robot must stand still)
    board fixed to a wrist, arm moves:    ref = <side>_wrist_roll_link
Solved in closed form (linear least squares), then refined by minimising the corner
reprojection error over all samples (Levenberg-Marquardt over the 12 pose parameters).
"""
import cv2
import numpy as np

# Head camera pose in head_pitch_link (optical frame: x right, y down, z forward), for comparison only
REFERENCES = {
    # Booster Studio MJCF (robot_t2.basset T2_v1p2_31dof_revo2.xml): <camera name="rgbd_camera"
    # pos="0.12 0 0.108" quat="0.499998 0.500002 -0.5 -0.5"> in head_pitch_link; MuJoCo cameras look
    # along -z with y up, so optical = camera * Rx(180deg)
    "studio": ("mujoco_camera", (0.12, 0.0, 0.108), (0.499998, 0.500002, -0.5, -0.5)),
    # t2_description/urdf/t2.urdf.xacro: head_booster_stereo_rgb_joint (+ identity head_color_optical_joint)
    "urdf": ("xyz_rpy", (0.08684, 0.0349, 0.08245), (-2.1792181, 0.0, -1.57079633)),
}


# ---------------------------------------------------------------- SE(3) helpers
def pose(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.ravel(t)
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return pose(R.T, -R.T @ t)


def from_rvec(rvec, tvec):
    return pose(cv2.Rodrigues(np.asarray(rvec, float).reshape(3))[0], tvec)


def to_rvec(T):
    return cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3].copy()


def quat_wxyz_to_R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_to_quat_xyzw(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def rpy_to_R(rpy):
    """URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    pitch = np.arcsin(-np.clip(R[2, 0], -1, 1))
    return np.array([np.arctan2(R[2, 1], R[2, 2]), pitch, np.arctan2(R[1, 0], R[0, 0])])


def angle_deg(R):
    return np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def reference(name):
    kind, xyz, rot = REFERENCES[name]
    if kind == "mujoco_camera":
        return pose(quat_wxyz_to_R(rot) @ np.diag([1, -1, -1]), xyz)
    return pose(rpy_to_R(rot), xyz)


def describe(T):
    return {"xyz": T[:3, 3].tolist(), "rpy": R_to_rpy(T[:3, :3]).tolist(),
            "quat_xyzw": R_to_quat_xyzw(T[:3, :3]).tolist(), "matrix": T.tolist()}


# ---------------------------------------------------------------- calibration
def pnp(corners, object_points, K, D):
    """T_cam_board and its RMS reprojection error [px]."""
    ok, rvec, tvec = cv2.solvePnP(object_points, corners, K, D, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(object_points, corners, K, D)
    rvec, tvec = cv2.solvePnPRefineLM(object_points, corners, K, D, rvec, tvec)
    projected = cv2.projectPoints(object_points, rvec, tvec, K, D)[0].reshape(-1, 2)
    return from_rvec(rvec, tvec), float(np.sqrt(np.mean(np.sum((projected - corners) ** 2, 1))))


MAX_OBLIQUE_DEG = 50.0      # board seen more edge-on than this: corners get unreliable (tests)
MAX_PNP_RMS_PX = 1.0


def view_problem(T_cam_board, pnp_rms):
    """Why one board view should not be used, or '' if it is fine."""
    ray = T_cam_board[:3, 3] / np.linalg.norm(T_cam_board[:3, 3])
    oblique = np.degrees(np.arccos(min(1.0, abs(T_cam_board[:3, 2] @ ray))))
    if oblique > MAX_OBLIQUE_DEG:
        return f"board too oblique ({oblique:.0f} deg > {MAX_OBLIQUE_DEG:.0f}): turn it towards the camera"
    if pnp_rms > MAX_PNP_RMS_PX:
        return f"corner fit poor ({pnp_rms:.2f} px > {MAX_PNP_RMS_PX}): bring the board closer / face the camera"
    return ""


def closed_form(G, C):
    """X = T_mount_cam, Y = T_ref_board from X C_i = G_i Y, linear least squares (Kronecker form).

    rotation:     Rx Rc_i = Rg_i Ry   ->  (Rc_i^T kron I) vec(Rx) - (I kron Rg_i) vec(Ry) = 0
    translation:  Rx tc_i + tx = Rg_i ty + tg_i   ->  [I  -Rg_i] [tx; ty] = tg_i - Rx tc_i
    """
    I = np.eye(3)
    A = np.vstack([np.hstack([np.kron(c[:3, :3].T, I), -np.kron(I, g[:3, :3])]) for g, c in zip(G, C)])
    v = np.linalg.svd(A)[2][-1]
    Rx, Ry = v[:9].reshape(3, 3, order="F"), v[9:].reshape(3, 3, order="F")
    scale = np.cbrt(np.linalg.det(Rx))
    Rx, Ry = _so3(Rx / scale), _so3(Ry / scale)
    M = np.vstack([np.hstack([I, -g[:3, :3]]) for g in G])
    b = np.concatenate([g[:3, 3] - Rx @ c[:3, 3] for g, c in zip(G, C)])
    t = np.linalg.lstsq(M, b, rcond=None)[0]
    return pose(Rx, t[:3]), pose(Ry, t[3:])


def _so3(M):
    U, _, Vt = np.linalg.svd(M)
    return U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt


def _residuals(params, G, corners, object_points, K, D):
    X, Y = from_rvec(params[0:3], params[3:6]), from_rvec(params[6:9], params[9:12])
    out = []
    for g, uv in zip(G, corners):
        rvec, tvec = to_rvec(inv(X) @ g @ Y)               # T_cam_board predicted by the kinematics
        out.append((cv2.projectPoints(object_points, rvec, tvec, K, D)[0].reshape(-1, 2) - uv).ravel())
    return np.concatenate(out)


def refine(X, Y, G, corners, object_points, K, D, iterations=50):
    """Levenberg-Marquardt on the corner reprojection error over all samples."""
    params = np.concatenate([*to_rvec(X), *to_rvec(Y)])
    r = _residuals(params, G, corners, object_points, K, D)
    cost, lam = r @ r, 1e-3
    for _ in range(iterations):
        J = np.empty((len(r), 12))
        for k in range(12):
            step = 1e-6
            d = params.copy()
            d[k] += step
            J[:, k] = (_residuals(d, G, corners, object_points, K, D) - r) / step
        A, g = J.T @ J, J.T @ r
        while True:
            delta = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            trial = params + delta
            r_trial = _residuals(trial, G, corners, object_points, K, D)
            if r_trial @ r_trial < cost:
                params, r, lam = trial, r_trial, lam / 3
                break
            lam *= 4
            if lam > 1e8:
                break
        new_cost = r @ r
        if lam > 1e8 or abs(cost - new_cost) < 1e-10 * max(cost, 1e-12):
            cost = new_cost
            break
        cost = new_cost
    return from_rvec(params[0:3], params[3:6]), from_rvec(params[6:9], params[9:12])


def per_sample_rms(X, Y, G, corners, object_points, K, D):
    out = []
    for g, uv in zip(G, corners):
        rvec, tvec = to_rvec(inv(X) @ g @ Y)
        e = cv2.projectPoints(object_points, rvec, tvec, K, D)[0].reshape(-1, 2) - uv
        out.append(float(np.sqrt(np.mean(np.sum(e ** 2, 1)))))
    return np.array(out)


def motion_spread_deg(G):
    """Largest rotation between any two samples' G (needs to be well above 0 on two axes)."""
    return max((angle_deg(a[:3, :3].T @ b[:3, :3]) for a in G for b in G), default=0.0)


def calibrate(G, corners, object_points, K, D, reject=True):
    """Full solve. Returns dict with X, Y, initial/refined per-sample RMS, and the indices used.

    reject: drop the worst sample and solve again while it is far off the rest (> 3x median and
    > 1.5 px), keeping at least 6. A misnumbered corner set is consistent in itself (small PnP error)
    but disagrees with every other sample, which only shows up here."""
    used = list(range(len(G)))
    while True:
        g, uv = [G[i] for i in used], [corners[i] for i in used]
        C = [pnp(c, object_points, K, D)[0] for c in uv]
        X0, Y0 = closed_form(g, C)
        X, Y = refine(X0, Y0, g, uv, object_points, K, D)
        rms = per_sample_rms(X, Y, g, uv, object_points, K, D)
        worst = int(np.argmax(rms))
        if not reject or len(used) <= 6 or rms[worst] <= max(3 * np.median(rms), 1.5):
            break
        used.pop(worst)
    return {"X": X, "Y": Y, "X0": X0, "used": used,
            "rms0": per_sample_rms(X0, Y0, g, uv, object_points, K, D), "rms": rms}
