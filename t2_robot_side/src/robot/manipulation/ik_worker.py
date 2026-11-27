"""Arm IK in a separate Python process.

Inside the agent process the IK solver is starved for the GIL by the relay callbacks (joint states,
TF and odometry arrive at ~500 Hz each): a solve that takes 0.2 s alone took minutes there. This
module runs arm_ik in its own interpreter (own GIL) and talks JSON lines over stdin / stdout.

    ik = IKWorker()                                     # starts the process on first use
    q, info = ik.solve("right", point, pos, R, q0)
    path = ik.line("right", point, q_start, pos, R, speed=0.06, dt=0.02)   # list of q (straight line)
Errors: IKError (out of reach, path leaves the workspace / needs a joint jump), RuntimeError (worker died).
"""
import json
import os
import subprocess
import sys
import threading

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


class IKError(RuntimeError):
    pass


# ---------------------------------------------------------------- worker side (separate process)
def _min_jerk(s):
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def _serve():
    sys.path.insert(0, HERE)
    from arm_ik import ArmIK
    cache = {}

    def get(side, point):
        key = (side, tuple(point))
        if key not in cache:
            cache[key] = ArmIK(side, point=tuple(point))
        return cache[key]

    for line in sys.stdin:
        try:
            r = json.loads(line)
            ik = get(r["side"], r["point"])
            R = None if r.get("R") is None else np.array(r["R"])
            if r["op"] == "solve":
                q, info = ik.solve(np.array(r["pos"]), R=R, q0=np.array(r["q0"]))
                out = {"q": q.tolist(), "info": {k: (bool(v) if k == "ok" else float(v)) for k, v in info.items()}}
            elif r["op"] == "line":
                q = np.array(r["q0"])
                p0, R0 = ik.fk(q)
                R = R0 if R is None else R
                pos = np.array(r["pos"])
                dist = float(np.linalg.norm(pos - p0))
                n = max(1, int(np.ceil(dist / (r["speed"] * r["dt"]))))
                path, prev = [], q
                for k in range(1, n + 1):
                    p = p0 + _min_jerk(k / n) * (pos - p0)
                    q, info = ik.solve(p, R=R, q0=q, restarts=0, iters=50)
                    if not info["ok"]:
                        raise IKError(f"line path leaves the workspace at {np.round(p, 3).tolist()}")
                    if np.max(np.abs(q - prev)) > 0.2:
                        raise IKError("line path needs a joint jump (singularity / limit)")
                    path.append(q.tolist())
                    prev = q
                out = {"path": path}
            else:
                raise ValueError(f"unknown op {r['op']!r}")
        except IKError as error:
            out = {"ik_error": str(error)}
        except Exception as error:
            out = {"error": f"{type(error).__name__}: {error}"}
        sys.stdout.write(json.dumps(out) + "\n")
        sys.stdout.flush()


# ---------------------------------------------------------------- agent side
class IKWorker:
    def __init__(self):
        self._proc = None
        self._lock = threading.Lock()

    def _call(self, request):
        with self._lock:
            if self._proc is None or self._proc.poll() is not None:
                env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
                self._proc = subprocess.Popen([sys.executable, "-u", os.path.abspath(__file__)],
                                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, env=env)
            self._proc.stdin.write(json.dumps(request) + "\n")
            self._proc.stdin.flush()
            line = self._proc.stdout.readline()
        if not line:
            raise RuntimeError("IK worker process died")
        out = json.loads(line)
        if "ik_error" in out:
            raise IKError(out["ik_error"])
        if "error" in out:
            raise RuntimeError(f"IK worker: {out['error']}")
        return out

    @staticmethod
    def _req(op, side, point, pos, R, q0, **kw):
        return {"op": op, "side": side, "point": [float(v) for v in point], "pos": [float(v) for v in pos],
                "R": None if R is None else np.asarray(R, float).tolist(), "q0": np.asarray(q0, float).tolist(), **kw}

    def solve(self, side, point, pos, R, q0):
        out = self._call(self._req("solve", side, point, pos, R, q0))
        return np.array(out["q"]), out["info"]

    def line(self, side, point, q0, pos, R, speed, dt):
        out = self._call(self._req("line", side, point, pos, R, q0, speed=float(speed), dt=float(dt)))
        return [np.array(q) for q in out["path"]]

    def close(self):
        with self._lock:
            if self._proc is not None:
                self._proc.kill()
                self._proc = None


if __name__ == "__main__":
    _serve()
