#!/usr/bin/env python3
"""Build robot_t2_revo2.basset (Booster Studio user robot asset) from the builtin robot_t2.basset.

    python tools/make_revo2_basset.py \
        --builtin  "C:/Users/Administrator/AppData/Roaming/Booster Studio/simulator-library/assets/robot_t2.basset" \
        --revo2    "../T2_31dof_Revo2Hand" \
        --out      out/robot_t2_revo2.basset

Only the Python standard library is needed. With `--check` and mujoco==3.10.0 installed it also
compiles the result (same MuJoCo version as Booster Studio).

Why the hands are NOT children of <side>_wrist_roll_link
  Booster Studio's robot_controller reads/writes each robot as ONE contiguous qpos block
  (root free joint + the 31 T2 joints, physics/extensions/robot_controller/controller.py) and
  sanitize_robot_asset_spec() deletes every top-level body except the trunk.  Finger joints placed
  under the wrist would sit in the middle of that block.  So each hand is:
    * a body appended as the LAST child of `trunk`  -> its joints come after the 31 robot joints,
    * free to move relative to the trunk (3 slide + 1 ball joint),
    * glued to <side>_wrist_roll_link by a <weld> equality (relpose taken at qpos0, where the hand
      body is placed exactly on the wrist frame).
  The 31 robot joints, their actuators, names and order are untouched, so Studio still detects
  the model as "T2" and booster-motion controls it as before.

No finger actuators: Studio only accepts robot assets whose actuators are exactly the 31 T2 ones
(identify_robot_model on all actuator names). The 6 driven joints per hand
(<side>_thumb_metacarpal/thumb_proximal/index_proximal/middle_proximal/ring_proximal/pinky_proximal_joint)
are driven by the revo2_hand physics extension through a PD on qfrc_applied; distal joints follow
through <equality joint> couplings.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import io
import json
import os
import zipfile
import xml.etree.ElementTree as ET

ASSET_ID = "robot_t2_revo2"
# No builtin identity: Booster Studio's desktop rejects user packages that claim one
# ("user package must not claim builtin identity or default status"). Consequence: the asset can be
# added to a scene in the editor at runtime, but Studio will not save/load scenes that reference it
# (only builtin "Default Assets" can be persisted in .bscene files).
ROBOT_XML = "T2_v1p2_31dof_revo2.xml"
REVO2_MESH_DIR = "revo2_meshes"

FINGERS = ("index", "middle", "ring", "pinky")
COUPLED = {"thumb_distal_joint": ("thumb_proximal_joint", 1.0)}  # URDF <mimic>
COUPLED.update({f"{f}_distal_joint": (f"{f}_proximal_joint", 1.155) for f in FINGERS})
HAND_BASE_MASS = 0.03          # the builtin wrist_roll_link inertia already contains a hand shell
WELD_SOLREF = "0.004 1"        # Studio scenes run at 2 ms; timeconst >= 2*dt
WRIST_EXCLUDE = ("wrist_roll_link", "wrist_yaw_link", "wrist_pitch_link")


def _is_finger(name: str) -> bool:
    return any(k in name for k in ("thumb", "index", "middle", "ring", "pinky"))


def _read_zip_member(path_or_bytes, member: str) -> bytes:
    src = io.BytesIO(path_or_bytes) if isinstance(path_or_bytes, bytes) else path_or_bytes
    with zipfile.ZipFile(src) as z:
        return z.read(member)


def _write_zip(entries, compress: int) -> bytes:
    """Reproducible zip (fixed timestamps, fixed order)."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        for name, data in entries:
            info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = compress
            info.external_attr = 0o644 << 16
            z.writestr(info, data)
    return buf.getvalue()


def build_robot_xml(builtin_xml: bytes, revo2_xml_path: str) -> tuple[ET.Element, set[str]]:
    t2 = ET.fromstring(builtin_xml)
    rv = ET.parse(revo2_xml_path).getroot()
    t2_bodies = {b.get("name"): b for b in t2.iter("body")}
    rv_bodies = {b.get("name"): b for b in rv.iter("body")}
    rv_meshes = {m.get("name"): m for m in rv.find("asset").iter("mesh")}
    joint_attr = {j.get("name"): j for j in rv.iter("joint")}
    used_meshes: set[str] = set()

    t2.set("model", "T2_v1p2_31dof_revo2")
    asset = t2.find("asset")
    trunk = t2_bodies["trunk"]
    equality = t2.find("equality")
    if equality is None:
        equality = ET.Element("equality")
        t2.insert(list(t2).index(t2.find("actuator")), equality)
    contact = t2.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        t2.insert(list(t2).index(equality), contact)

    for side, sy in (("left", 1.0), ("right", -1.0)):
        # 1) builtin wrist: drop the static hand shell (visual) and its old palm collision
        wr = t2_bodies[f"{side}_wrist_roll_link"]
        for g in list(wr.findall("geom")):
            mesh = g.get("mesh", "")
            if mesh.endswith("_with_hand") or mesh.endswith("_wrist_roll_link_collision"):
                wr.remove(g)
        ET.SubElement(wr, "geom", {"class": "visual", "rgba": "0.847 0.847 0.847 1",
                                   "mesh": f"revo2_{side}_wrist_roll_link"})
        used_meshes.add(f"{side}_wrist_roll_link")

        # 2) hand body = everything under the Revo2 wrist_roll_link except its own joint/inertial/wrist mesh
        src = rv_bodies[f"{side}_wrist_roll_link"]
        hand = ET.Element("body", {"name": f"{side}_revo2_hand", "pos": "0 0 0", "quat": "1 0 0 0"})
        ET.SubElement(hand, "inertial", {"pos": f"0 {sy*0.11:.3f} 0", "mass": str(HAND_BASE_MASS),
                                         "diaginertia": "3e-5 3e-5 3e-5"})
        for ax, name in (("1 0 0", "tx"), ("0 1 0", "ty"), ("0 0 1", "tz")):
            ET.SubElement(hand, "joint", {"name": f"{side}_revo2_hand_{name}", "type": "slide", "axis": ax,
                                          "limited": "false", "armature": "0.001", "damping": "0.01"})
        ET.SubElement(hand, "joint", {"name": f"{side}_revo2_hand_rot", "type": "ball",
                                      "limited": "false", "armature": "0.0005", "damping": "0.002"})
        for child in src:
            if child.tag in ("inertial", "joint"):
                continue
            if child.tag == "geom" and child.get("mesh") == f"{side}_wrist_roll_link":
                continue  # wrist mesh stays on the wrist body
            c = copy.deepcopy(child)
            if c.tag == "geom" and c.get("name") == f"{side}_wrist_roll_link_collision":
                c.set("name", f"{side}_revo2_palm_collision")
            hand.append(c)
        # finger joints: damping, finger collision: friction
        for j in hand.iter("joint"):
            if _is_finger(j.get("name", "")):
                j.set("damping", "0.02")
                j.set("armature", "0.0005")
        for g in hand.iter("geom"):
            if g.get("mesh"):
                used_meshes.add(g.get("mesh"))
                g.set("mesh", "revo2_" + g.get("mesh"))
            if g.get("class") == "collision":
                g.set("friction", "1.5 0.02 0.002")
                g.set("condim", "4")
        # sites (hand frame = wrist_roll frame: fingers along -y right / +y left, palm normal -z, thumb +x)
        ET.SubElement(hand, "site", {"name": f"{side}_palm", "pos": f"0 {sy*0.12:.3f} -0.012", "size": "0.008",
                                     "rgba": "1 0 0 0.6", "group": "4"})
        ET.SubElement(hand, "site", {"name": f"{side}_grasp", "pos": f"0 {sy*0.13:.3f} -0.05", "size": "0.008",
                                     "rgba": "0 1 0 0.6", "group": "4"})
        trunk.append(hand)  # appended last -> its joints come after the 31 robot joints
        # hand placed on the wrist frame at qpos0: pos/quat relative to trunk computed by caller
        hand.set("__wrist__", f"{side}_wrist_roll_link")

        # 3) weld + contact excludes
        ET.SubElement(equality, "weld", {"name": f"{side}_revo2_hand_weld", "body1": f"{side}_revo2_hand",
                                         "body2": f"{side}_wrist_roll_link", "solref": WELD_SOLREF})
        hand_bodies = [hand] + list(hand.iter("body"))[1:]
        for hb in hand_bodies:
            for w in WRIST_EXCLUDE:
                ET.SubElement(contact, "exclude", {"body1": hb.get("name"), "body2": f"{side}_{w}"})

        # 4) finger couplings (URDF <mimic>). NO actuators: Studio's robot-asset check
        #    (core/asset_package_models.load_robot_asset_package_bytes -> identify_robot_model)
        #    requires the asset to have exactly the 31 T2 actuators. The revo2_hand extension
        #    drives the 6 driven finger joints with a PD on qfrc_applied instead.
        for dist, (prox, k) in COUPLED.items():
            ET.SubElement(equality, "joint", {"name": f"{side}_{dist.replace('_joint', '')}_couple",
                                              "joint1": f"{side}_{dist}", "joint2": f"{side}_{prox}",
                                              "polycoef": f"0 {k} 0 0 0", "solref": "0.005 1"})

    # meshes
    for name in sorted(used_meshes):
        m = rv_meshes[name]
        ET.SubElement(asset, "mesh", {"name": "revo2_" + name, "file": f"{REVO2_MESH_DIR}/{m.get('file')}"})
    # builtin hand-shell meshes no longer used
    for m in list(asset.findall("mesh")):
        n = m.get("name", "")
        if n.endswith("_with_hand") or n.endswith("_wrist_roll_link_collision"):
            asset.remove(m)
    return t2, used_meshes


def _quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return (w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2)


def _rot(q, v):
    w, x, y, z = q
    qv = (0.0, v[0], v[1], v[2])
    qc = (w, -x, -y, -z)
    r = _quat_mul(_quat_mul(q, qv), qc)
    return (r[1], r[2], r[3])


def _parse(v, n):
    return tuple(float(x) for x in v.split()) if v else None


def place_hands_on_wrists(root: ET.Element) -> None:
    """Set each hand body's pos/quat (relative to trunk) = wrist_roll_link pose at qpos0.

    Pure chain of body pos/quat from trunk to the wrist (all joint angles 0).
    The builtin XML uses pos + quat only (no euler/axisangle) on these bodies.
    """
    parent = {}
    for b in root.iter("body"):
        for c in b.findall("body"):
            parent[c] = b
    bodies = {b.get("name"): b for b in root.iter("body")}
    trunk = bodies["trunk"]
    for hand in [b for b in trunk.findall("body") if b.get("__wrist__")]:
        wrist = bodies[hand.attrib.pop("__wrist__")]
        chain = []
        b = wrist
        while b is not trunk:
            chain.append(b)
            b = parent[b]
        p, q = (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
        for b in reversed(chain):
            for bad in ("euler", "axisangle", "xyaxes", "zaxis"):
                if b.get(bad):
                    raise ValueError(f"body {b.get('name')} uses {bad}; only pos/quat supported")
            bp = _parse(b.get("pos"), 3) or (0.0, 0.0, 0.0)
            bq = _parse(b.get("quat"), 4) or (1.0, 0.0, 0.0, 0.0)
            r = _rot(q, bp)
            p = (p[0] + r[0], p[1] + r[1], p[2] + r[2])
            q = _quat_mul(q, bq)
        n = sum(c * c for c in q) ** 0.5
        q = tuple(c / n for c in q)
        hand.set("pos", " ".join(f"{v:.8g}" for v in p))
        hand.set("quat", " ".join(f"{v:.8g}" for v in q))


def build(builtin_basset: str, revo2_dir: str, out_path: str, check: bool = False) -> str:
    with zipfile.ZipFile(builtin_basset) as z:
        manifest = json.loads(z.read("manifest.json"))
        inner = z.read("model.mjspec.zip")
        thumb_path = manifest["thumbnail"]["path"]
        thumb = z.read(thumb_path)
    if manifest.get("id") != "robot_t2":
        raise SystemExit(f"expected the builtin robot_t2.basset, got id={manifest.get('id')}")
    with zipfile.ZipFile(io.BytesIO(inner)) as z:
        names = z.namelist()
        xml_names = [n for n in names if n.endswith(".xml")]
        builtin_xml = z.read(xml_names[0])
        t2_meshes = [(n, z.read(n)) for n in names if not n.endswith(".xml")]

    revo2_xml = os.path.join(revo2_dir, "T2_31dof_Revo2Hand.xml")
    root, used = build_robot_xml(builtin_xml, revo2_xml)
    place_hands_on_wrists(root)
    ET.indent(root, "  ")
    xml_bytes = ET.tostring(root, encoding="utf-8")

    rv_mesh_files = {m.get("name"): m.get("file") for m in ET.parse(revo2_xml).getroot().find("asset").iter("mesh")}
    keep_t2 = {m.get("file") for m in root.find("asset").iter("mesh") if not m.get("file", "").startswith(REVO2_MESH_DIR)}
    entries = [(n, d) for n, d in t2_meshes if n in keep_t2]
    for name in sorted(used):
        f = rv_mesh_files[name]
        with open(os.path.join(revo2_dir, "meshes", f), "rb") as fh:
            entries.append((f"{REVO2_MESH_DIR}/{f}", fh.read()))
    entries.sort()
    entries.append((ROBOT_XML, xml_bytes))
    model_zip = _write_zip(entries, zipfile.ZIP_STORED)

    m = dict(manifest)
    for k in ("builtin_id", "builtin_revision", "created_at"):
        m.pop(k, None)
    m.update({
        "id": ASSET_ID,
        "name": "T2 Revo2",
        "origin": "user",
        "deletable": True,
        "author": "robo@home",
        "description": "T2 with two BrainCo Revo2 hands (6 actuated DOF each); needs the revo2_hand physics extension",
        "tags": ["robot", "t2", "humanoid", "revo2", "dexterous_hand"],
    })
    m["thumbnail"] = {"path": thumb_path, "mime_type": manifest["thumbnail"]["mime_type"],
                      "sha256": hashlib.sha256(thumb).hexdigest()}
    out = _write_zip([("manifest.json", json.dumps(m, ensure_ascii=False, indent=2).encode()),
                      ("model.mjspec.zip", model_zip), (thumb_path, thumb)], zipfile.ZIP_DEFLATED)
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "wb") as f:
        f.write(out)
    print(f"wrote {out_path}  ({len(out)/1e6:.1f} MB, sha256 {hashlib.sha256(out).hexdigest()})")
    if check:
        check_basset(out_path)
    return out_path


def check_basset(path: str) -> None:
    import mujoco
    import numpy as np
    inner = _read_zip_member(open(path, "rb"), "model.mjspec.zip")
    spec = mujoco.MjSpec.from_zip(io.BytesIO(inner))
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    names = [model.joint(i).name for i in range(model.njnt)]
    acts = [model.actuator(i).name for i in range(model.nu)]
    print(f"mujoco {mujoco.__version__}: nq={model.nq} nu={model.nu} neq={model.neq} joints[1:32] contiguous:",
          all(not _is_finger(n) and "revo2" not in n for n in names[1:32]))
    for side in ("left", "right"):
        h, w = model.body(f"{side}_revo2_hand").id, model.body(f"{side}_wrist_roll_link").id
        print(f"  {side}: hand-wrist offset {np.linalg.norm(data.xpos[h]-data.xpos[w])*1000:.3f} mm, "
              f"rot err {np.abs(data.xmat[h]-data.xmat[w]).max():.2e}")
    print("  actuators:", len(acts), "(must be 31)")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    t2 = os.path.dirname(os.path.dirname(here))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--builtin", default=os.path.expanduser(
        "~/AppData/Roaming/Booster Studio/simulator-library/assets/robot_t2.basset"))
    ap.add_argument("--revo2", default=os.path.join(t2, "T2_31dof_Revo2Hand"))
    ap.add_argument("--out", default=os.path.join(os.path.dirname(here), "out", f"{ASSET_ID}.basset"))
    ap.add_argument("--check", action="store_true", help="compile with the installed mujoco (use 3.10.0)")
    a = ap.parse_args()
    build(a.builtin, a.revo2, a.out, a.check)


if __name__ == "__main__":
    main()
