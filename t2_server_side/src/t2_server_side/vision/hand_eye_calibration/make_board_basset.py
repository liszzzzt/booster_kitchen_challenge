#!/usr/bin/env python3
"""Make the calibration board: a Booster Studio prop (calib_board.basset) and a printable PNG.

    python3 make_board_basset.py [--out output]
      -> output/calib_board.basset      copy to %APPDATA%\\Booster Studio\\simulator-library\\assets\\ (Windows)
                                        or ~/.config/Booster Studio/simulator-library/assets/, restart Studio,
                                        then add "Calibration Board" in the scene editor
      -> output/calib_board_print.png   300 dpi, 24 mm squares (--print-square): print at 100 % on A4 landscape,
                                        then measure a square and pass it as --square

The prop is a floor stand: board centre 1.2 m high (about the head camera's height), leaning back 10 deg,
front facing the body's +x. Set it to static in the editor and turn its front towards the robot. The board pattern
is board.py (10 x 7 squares of 40 mm, 30 mm white border).
"""
import argparse
import hashlib
import io
import json
import os
import time
import zipfile

import cv2
import numpy as np

import board

HEIGHT = 1.20                        # board centre above the floor [m], about T2 head-camera height
TILT = np.radians(10.0)              # board leans back by this much from vertical
THICK = 0.01                         # board thickness [m]
FOOT = (0.40, 0.40, 0.02)            # foot plate x, y, z size [m]
POLE = 0.03                          # square pole edge [m]


def mjcf():
    w, h = board.SIZE
    s, c = np.sin(TILT), np.cos(TILT)
    # board frame: x to the viewer's right (+y body), y up the board, z out of the face (front, +x tilted up)
    R = np.array([[0, -s, c], [1, 0, 0], [0, c, s]])           # columns: x, y, z axes in the body frame
    center = np.array([0.0, 0.0, HEIGHT])
    face = center + R[:, 2] * (THICK / 2 + 0.0005)
    pole_top = center - R[:, 2] * (THICK / 2 + POLE / 2)
    pole = (pole_top + np.array([pole_top[0], 0, FOOT[2]])) / 2      # vertical pole from the foot to the board
    quat = cv2.Rodrigues(R)[0].ravel()
    angle = np.linalg.norm(quat)
    axis = quat / angle
    q = np.concatenate([[np.cos(angle / 2)], axis * np.sin(angle / 2)])
    f = lambda v: " ".join(f"{x:.6g}" for x in v)
    return f"""<mujoco model="calib_board">
  <compiler angle="radian"/>
  <asset>
    <texture type="2d" name="calib_board" file="calib_board.png"/>
    <material name="calib_board" texture="calib_board" specular="0" shininess="0" reflectance="0"/>
    <mesh name="calib_board_face" file="calib_board_face.obj" inertia="shell"/>
  </asset>
  <worldbody>
    <body name="calib_board">
      <joint name="calib_board_free" type="free" damping="0.1"/>
      <geom name="calib_board_foot" type="box" size="{f(np.array(FOOT) / 2)}" pos="0 0 {FOOT[2] / 2:.6g}" rgba="0.3 0.3 0.3 1"/>
      <geom name="calib_board_pole" type="box" size="{POLE / 2} {POLE / 2} {(pole_top[2] - FOOT[2]) / 2:.6g}" pos="{f(pole)}" rgba="0.3 0.3 0.3 1"/>
      <geom name="calib_board_panel" type="box" size="{w / 2:.6g} {h / 2:.6g} {THICK / 2:.6g}" pos="{f(center)}" quat="{f(q)}" rgba="0.85 0.85 0.85 1"/>
      <geom name="calib_board_face" type="mesh" mesh="calib_board_face" material="calib_board" pos="{f(face)}" quat="{f(q)}" contype="0" conaffinity="0" mass="0"/>
    </body>
  </worldbody>
</mujoco>
"""


def face_obj():
    w, h = board.SIZE
    return (f"v {-w / 2} {-h / 2} 0\nv {w / 2} {-h / 2} 0\nv {w / 2} {h / 2} 0\nv {-w / 2} {h / 2} 0\n"
            "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n"
            "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")


def manifest(thumbnail):
    w, h = board.SIZE
    return {
        "schema_version": 1, "id": "calib_board", "name": "Calibration Board", "type": "asset",
        "asset_type": "prop", "origin": "user", "deletable": True, "version": "1.0.0", "author": "robo@home",
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "description": f"Checkerboard {board.SQUARES[0]}x{board.SQUARES[1]} squares of {board.SQUARE * 1000:.0f} mm "
                       "for head-camera hand-eye calibration",
        "tags": ["prop", "calibration", "checkerboard"],
        "mjspec_zip": "model.mjspec.zip",
        "sceneEdit": {"op": "attach_prop", "params": {"movable": True}},
        "editor": {
            "selection": {"rootBody": "calib_board"},
            "capabilities": {"motion": True, "resize": False, "resizeMode": "none", "color": False,
                             "friction": False, "mass": False},
            "defaults": {"motion": "physics-driven"},
            "resize": {"mode": "none"},
            "import": {"sourceType": "mjcf", "collision": "auto"},
            "placement": {"bottomOffset": 0.0,
                          "previewDimensions": [FOOT[0], max(FOOT[1], w), HEIGHT + h / 2]},
        },
        "thumbnail": {"path": "thumbnail.webp", "mime_type": "image/webp",
                      "sha256": hashlib.sha256(thumbnail).hexdigest()},
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--print-square", type=float, default=0.024, help="square size of the printout [m]")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "output"))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    png = cv2.imencode(".png", board.texture(64))[1].tobytes()
    thumbnail = cv2.imencode(".webp", board.texture(16))[1].tobytes()
    model = io.BytesIO()
    with zipfile.ZipFile(model, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("calib_board.xml", mjcf())
        z.writestr("calib_board.png", png)
        z.writestr("calib_board_face.obj", face_obj())
    path = os.path.join(a.out, "calib_board.basset")
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("manifest.json", json.dumps(manifest(thumbnail), indent=2))
        z.writestr("model.mjspec.zip", model.getvalue())
        z.writestr("thumbnail.webp", thumbnail)
    print("wrote", path)

    dots_per_m = 300 / 0.0254
    px = round(a.print_square * dots_per_m)
    print_path = os.path.join(a.out, "calib_board_print.png")
    image = board.texture(px)
    cv2.imwrite(print_path, image)
    h, w = np.array(image.shape) / dots_per_m * 1000
    print(f"wrote {print_path} (300 dpi: {w:.0f} x {h:.0f} mm, square {a.print_square * 1000:.0f} mm)")


if __name__ == "__main__":
    main()
