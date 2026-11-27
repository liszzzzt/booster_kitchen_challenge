"""Pick and place from the command line (T2WS; the T2 agent must be running, robot in front of the table).

    cd /ws/t2_server_side/src
    python3 -m t2_server_side.manipulation.pick_place_cli locate cup              # where is it (trunk frame)
    python3 -m t2_server_side.manipulation.pick_place_cli run --label cup --shift 0 0.08 0
    python3 -m t2_server_side.manipulation.pick_place_cli run --xyz 0.40 -0.08 -0.10 --to 0.40 -0.02 -0.10

run = lock legs -> (locate) -> pick -> place -> release. Place target: --to x y z, or --shift dx dy dz
from the pick position (default: put it back where it was). --keep: do not release at the end.
Single steps: start | pick --xyz ... | place --xyz ... | release.
Positions: trunk frame, cylinder centre at grasp height; --radius = cylinder radius (default 0.035).
Right hand reach: x 0.35..0.45, y -0.14..0.00, z -0.15..-0.05 m (left hand: y mirrored).
"""
import argparse
import threading

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor

from t2_server_side.manipulation.manager import ManipulationError, ManipulationManager
from t2_server_side.ros.node import T2ServerNode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["locate", "run", "start", "pick", "place", "release"])
    ap.add_argument("label_arg", nargs="?", help="label for 'locate'")
    ap.add_argument("--label", help="locate this object with vision (run / pick)")
    ap.add_argument("--xyz", type=float, nargs=3, help="object centre, trunk frame [m]")
    ap.add_argument("--to", type=float, nargs=3, help="place target, trunk frame [m]")
    ap.add_argument("--shift", type=float, nargs=3, default=(0.0, 0.0, 0.0), help="place = pick + shift [m]")
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--radius", type=float, default=0.035)
    ap.add_argument("--head-pitch", type=float, default=0.35)
    ap.add_argument("--leg-gain-scale", type=float, default=3.0, help="stiffness of the locked legs (x example gains)")
    ap.add_argument("--keep", action="store_true", help="run: keep the legs locked at the end")
    a = ap.parse_args()

    rclpy.init()
    node = T2ServerNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    mm = ManipulationManager(node)
    try:
        if a.step == "locate":
            label = a.label_arg or a.label
            if not label:
                raise SystemExit("locate needs a label, e.g. locate cup")
            p = mm.locate(label, a.radius)
            print(f"{label}: [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] m (trunk, cylinder axis)")
            return
        if not node.wait_for_robot():
            raise SystemExit("Robot side is not available (T2 agent running?)")
        if a.step == "start":
            mm.start(a.head_pitch, a.leg_gain_scale)
        elif a.step == "release":
            mm.release()
        elif a.step == "place":
            mm.place(a.side, a.to or a.xyz)
        else:                                                    # pick / run
            mm.start(a.head_pitch, a.leg_gain_scale)
            xyz = np.array(a.xyz) if a.xyz else (mm.locate(a.label, a.radius) if a.label else None)
            if xyz is None:
                raise SystemExit("give --xyz x y z or --label <name>")
            print(f"pick at [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]")
            mm.pick(a.side, xyz, a.radius)
            if a.step == "run":
                target = np.array(a.to) if a.to else xyz + np.array(a.shift)
                print(f"place at [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
                mm.place(a.side, target)
                if not a.keep:
                    mm.release()
        print("done")
    except ManipulationError as error:
        raise SystemExit(f"failed: {error}")
    finally:                       # stop spinning before destroying (else: "terminate called ...")
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        if mm.locator is not None:
            mm.locator.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
