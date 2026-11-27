#!/usr/bin/env python3
"""Put a hand-eye calibration checkerboard on the back of a Revo2 hand in the T2 robot asset.

    python tools/add_calib_board.py                         # both hands, builtin_replacement/robot_t2.basset in
                                                            #   -> out/robot_t2_calib.basset
    python tools/add_calib_board.py --side right --check    # one hand only; --check: compile with mujoco==3.10.0
    python tools/patch_builtin_t2.py apply --replacement out/robot_t2_calib.basset   # Studio closed
    ... calibrate (t2_server_side/.../hand_eye_calibration: collect.py --board hand) ...
    python tools/patch_builtin_t2.py apply                  # back to the normal T2 + Revo2

Why inside the robot: Studio scene props stay in the world and cannot be attached to the robot, so the
board has to be part of the robot model. It is added to <side>_revo2_hand, the hand body that the
revo2_hand extension keeps welded onto <side>_wrist_roll_link, so it is rigid in the wrist frame.
It is visual only (class "visual": no contacts, no mass), so walking and grasping physics are unchanged.

Board: 6 x 5 squares of 25 mm, 15 mm white border (180 x 155 mm), 5 x 4 inner corners; MUST match
board.BOARDS["hand"] in t2_server_side/src/t2_server_side/vision/hand_eye_calibration/board.py.
Placed on the back of the hand (+z of the wrist frame, the palm faces -z), centre (0, -+0.13, 0.03) m,
face towards +z, 180 mm side along the wrist x axis. The fingers curl towards the palm, away from it;
the grasp points (palm side) are not covered. The exact pose does not matter for the calibration
(solve.py estimates it), it only has to be rigid. Standard library only (mujoco only for --check).
"""
from __future__ import annotations

import argparse
import hashlib
import io
import os
import struct
import zipfile
import zlib
import xml.etree.ElementTree as ET

SQUARES = (6, 5)            # columns (along wrist x), rows (along wrist y)
SQUARE = 0.025
MARGIN = 0.015
CENTER = (0.0, -0.13, 0.03)  # right hand, wrist frame; left = y mirrored
THICK = 0.005
PX = 16                      # texture pixels per square; 64 aliased at 320x240 (corners misnumbered in tests)
FACE_GAP = 0.002             # textured face above the panel; 0.5 mm z-fought (patches of plain grey) in tests
FILES = "calib_board"        # folder inside model.mjspec.zip


def board_png() -> bytes:
    """Grey PNG of the board, top-left square black (same layout as board.Board.texture)."""
    m = round(PX * MARGIN / SQUARE)
    cols, rows = SQUARES
    w, h = cols * PX + 2 * m, rows * PX + 2 * m
    raw = bytearray()
    for y in range(h):
        raw.append(0)                                   # filter: none
        r = (y - m) // PX if m <= y < h - m else None
        for x in range(w):
            c = (x - m) // PX if m <= x < w - m else None
            raw.append(0 if r is not None and c is not None and (r + c) % 2 == 0 else 255)

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b""))


def face_obj(w: float, h: float) -> bytes:
    """Quad in its xy plane, normal +z, texture u along +x, v along +y."""
    return (f"v {-w / 2} {-h / 2} 0\nv {w / 2} {-h / 2} 0\nv {w / 2} {h / 2} 0\nv {-w / 2} {h / 2} 0\n"
            "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\nf 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n").encode()


def add_boards(root: ET.Element, sides: list[str]) -> None:
    if any("calib_board" in (g.get("name") or "") for g in root.iter("geom")):
        raise SystemExit("this asset already has a calibration board; start from the normal T2 + Revo2 asset")
    asset = root.find("asset")
    ET.SubElement(asset, "texture", {"name": "calib_board_hand", "type": "2d", "file": f"{FILES}/hand_board.png"})
    ET.SubElement(asset, "material", {"name": "calib_board_hand", "texture": "calib_board_hand",
                                      "specular": "0", "shininess": "0", "reflectance": "0"})
    ET.SubElement(asset, "mesh", {"name": "calib_board_hand_face", "file": f"{FILES}/hand_board_face.obj",
                                  "inertia": "shell"})
    for side in sides:
        add_board(root, side)


def add_board(root: ET.Element, side: str) -> None:
    hand = next((b for b in root.iter("body") if b.get("name") == f"{side}_revo2_hand"), None)
    if hand is None:
        raise SystemExit(f"no body {side}_revo2_hand: use the T2 + Revo2 asset (builtin_replacement/robot_t2.basset)")
    w, h = SQUARES[0] * SQUARE + 2 * MARGIN, SQUARES[1] * SQUARE + 2 * MARGIN
    x, y, z = CENTER[0], CENTER[1] if side == "right" else -CENTER[1], CENTER[2]
    f = lambda *v: " ".join(f"{a:.6g}" for a in v)
    ET.SubElement(hand, "geom", {"name": f"{side}_calib_board_panel", "class": "visual", "type": "box",
                                 "size": f(w / 2, h / 2, THICK / 2), "pos": f(x, y, z), "rgba": "0.85 0.85 0.85 1"})
    ET.SubElement(hand, "geom", {"name": f"{side}_calib_board_face", "class": "visual", "type": "mesh",
                                 "mesh": "calib_board_hand_face", "material": "calib_board_hand",
                                 "pos": f(x, y, z + THICK / 2 + FACE_GAP)})


def build(src: str, sides: list[str], out: str) -> str:
    with zipfile.ZipFile(src) as z:
        outer = {n: z.read(n) for n in z.namelist()}
    with zipfile.ZipFile(io.BytesIO(outer["model.mjspec.zip"])) as z:
        inner = {n: z.read(n) for n in z.namelist()}
    xml_name = next(n for n in inner if n.endswith(".xml"))
    root = ET.fromstring(inner[xml_name])
    add_boards(root, sides)
    ET.indent(root, "  ")
    inner[xml_name] = ET.tostring(root, encoding="utf-8")
    w, h = SQUARES[0] * SQUARE + 2 * MARGIN, SQUARES[1] * SQUARE + 2 * MARGIN
    inner[f"{FILES}/hand_board.png"] = board_png()
    inner[f"{FILES}/hand_board_face.obj"] = face_obj(w, h)

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:          # same as make_revo2_basset.py
        for n in sorted(inner):
            z.writestr(zipfile.ZipInfo(n, (1980, 1, 1, 0, 0, 0)), inner[n])
    outer["model.mjspec.zip"] = buf.getvalue()
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:        # manifest unchanged (builtin identity)
        for n, d in outer.items():
            info = zipfile.ZipInfo(n, (1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, d)
    data = open(out, "rb").read()
    print(f"wrote {out} ({len(data) / 1e6:.1f} MB, sha256 {hashlib.sha256(data).hexdigest()[:16]}...): "
          f"{' + '.join(sides)} hand, board {SQUARES[0]}x{SQUARES[1]} x {SQUARE * 1000:.0f} mm")
    return out


def check(path: str, sides: list[str]) -> None:
    import mujoco
    with zipfile.ZipFile(path) as z:
        spec = mujoco.MjSpec.from_zip(io.BytesIO(z.read("model.mjspec.zip")))
    model = spec.compile()
    print(f"compiled with mujoco {mujoco.__version__}: {model.nbody} bodies, {model.nu} actuators")
    for side in sides:
        g = model.geom(f"{side}_calib_board_face")
        print(f"  {side} board face on body {model.body(int(g.bodyid[0])).name}, contype {int(g.contype[0])}, "
              f"conaffinity {int(g.conaffinity[0])}")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="src", default=os.path.join(root, "builtin_replacement", "robot_t2.basset"),
                    help="T2 + Revo2 asset (builtin replacement)")
    ap.add_argument("--side", choices=["both", "right", "left"], default="both")
    ap.add_argument("--out", default=os.path.join(root, "out", "robot_t2_calib.basset"))
    ap.add_argument("--check", action="store_true", help="compile with the installed mujoco (use 3.10.0)")
    a = ap.parse_args()
    sides = ["right", "left"] if a.side == "both" else [a.side]
    build(a.src, sides, a.out)
    if a.check:
        check(a.out, sides)


if __name__ == "__main__":
    main()
