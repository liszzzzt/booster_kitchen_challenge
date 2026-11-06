#!/usr/bin/env python3
"""Replace Booster Studio's builtin T2 (robot_t2.basset) with the T2 + Revo2 version — or restore it.

    python tools/patch_builtin_t2.py apply     # close Booster Studio first
    python tools/patch_builtin_t2.py restore
    python tools/patch_builtin_t2.py status

Why: Studio only lets scenes persist builtin robot assets, so the only way to have every T2 scene
(e.g. kitchen_v3_T2) start with Revo2 hands is to change the builtin T2 itself. The replacement keeps
the original manifest (id robot_t2, builtin-asset-robot_t2, revision 2026.08.2), only the model differs.

It touches three files and backs each up once (never overwrites an existing backup):
  <seed>/assets/robot_t2.basset        seed = <Studio>/resources/app/booster-native/statics/virtual-robot/library-seed
  <seed>/seed-manifest.json            robot_t2 record: sha256 + size updated
  <library>/assets/robot_t2.basset     library = %APPDATA%/Booster Studio/simulator-library
Effects: ALL T2 scenes get Revo2 hands; a Studio update restores the original (run apply again).
The revo2_hand physics extension must still be installed in the virtual-robot container.
Standard library only.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEF_STUDIO = r"E:\Program Files\BoosterStudio"
DEF_LIBRARY = os.path.join(os.path.expanduser("~"), "AppData", "Roaming", "Booster Studio", "simulator-library")
ORIGINAL_SHA = "efebebf83ef9947087cfc8d059f81fcc7275555ab12698c686516fd36787582a"   # builtin rev 2026.08.2


def sha(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def paths(a):
    seed = a.seed or os.path.join(a.studio, "resources", "app", "booster-native", "statics", "virtual-robot", "library-seed")
    return {
        "seed_basset": os.path.join(seed, "assets", "robot_t2.basset"),
        "seed_manifest": os.path.join(seed, "seed-manifest.json"),
        "lib_basset": os.path.join(a.library, "assets", "robot_t2.basset"),
    }


def backup_name(key: str) -> str:
    return {"seed_basset": "seed_robot_t2.basset", "seed_manifest": "seed-manifest.json",
            "lib_basset": "library_robot_t2.basset"}[key]


def update_seed_manifest(path: str, new_sha: str, new_size: int) -> None:
    raw = open(path, "rb").read()
    m = json.loads(raw)
    recs = [r for r in m["assets"] if r.get("preferred_key") == "robot_t2"]
    if len(recs) != 1:
        sys.exit("seed-manifest.json: expected exactly one robot_t2 record")
    recs[0]["sha256"], recs[0]["size"] = new_sha, new_size
    text = json.dumps(m, ensure_ascii=False, indent=2)
    if raw.endswith(b"\n"):
        text += "\n"
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def status(p) -> None:
    for k, v in p.items():
        if k.endswith("basset"):
            s = sha(v) if os.path.exists(v) else "missing"
            print(f"{k:14s} {s[:16]}…  {'ORIGINAL' if s == ORIGINAL_SHA else 'MODIFIED'}  {v}")
    m = json.load(open(p["seed_manifest"], encoding="utf-8"))
    r = [x for x in m["assets"] if x.get("preferred_key") == "robot_t2"][0]
    print(f"seed manifest  robot_t2 sha256 {r['sha256'][:16]}… size {r['size']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["apply", "restore", "status"])
    ap.add_argument("--studio", default=DEF_STUDIO, help="Booster Studio install dir")
    ap.add_argument("--seed", default=None, help="library-seed dir (overrides --studio)")
    ap.add_argument("--library", default=DEF_LIBRARY)
    default_rep = next((c for c in (os.path.join(ROOT, "builtin_replacement", "robot_t2.basset"),
                                    os.path.join(ROOT, "out", "builtin_replacement", "robot_t2.basset"))
                        if os.path.exists(c)), os.path.join(ROOT, "builtin_replacement", "robot_t2.basset"))
    ap.add_argument("--replacement", default=default_rep)
    ap.add_argument("--backup-dir", default=os.path.join(ROOT, "builtin_backup"))
    a = ap.parse_args()
    p = paths(a)
    for k, v in p.items():
        if not os.path.exists(v):
            sys.exit(f"not found: {v}")

    if a.action == "status":
        status(p)
        return

    if a.action == "apply":
        os.makedirs(a.backup_dir, exist_ok=True)
        for k, v in p.items():
            dst = os.path.join(a.backup_dir, backup_name(k))
            if not os.path.exists(dst):
                if k.endswith("basset") and sha(v) != ORIGINAL_SHA:
                    sys.exit(f"{v} is not the original builtin T2 and no backup exists; refusing")
                shutil.copy2(v, dst)
                print("backup", v, "->", dst)
        new_sha, new_size = sha(a.replacement), os.path.getsize(a.replacement)
        shutil.copyfile(a.replacement, p["seed_basset"])
        shutil.copyfile(a.replacement, p["lib_basset"])
        update_seed_manifest(p["seed_manifest"], new_sha, new_size)
        print(f"applied: robot_t2.basset -> T2 + Revo2 (sha256 {new_sha[:16]}…, {new_size} bytes)")
    else:
        for k, v in p.items():
            src = os.path.join(a.backup_dir, backup_name(k))
            if not os.path.exists(src):
                sys.exit(f"backup missing: {src}")
            shutil.copyfile(src, v)
            print("restored", v)
    status(p)
    print("Restart Booster Studio.")


if __name__ == "__main__":
    main()
