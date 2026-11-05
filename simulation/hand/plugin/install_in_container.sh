#!/usr/bin/env bash
# Install / uninstall the revo2_hand physics extension INSIDE the Booster Studio virtual-robot container.
#   bash /tmp/revo2_plugin/install_in_container.sh            # install (idempotent)
#   bash /tmp/revo2_plugin/install_in_container.sh uninstall  # restore builtin.py, remove the extension
# Afterwards restart the container (docker restart <name>) so bs-phys-worker re-imports the registry.
set -euo pipefail
SIM=${BOOSTER_SIM_DIR:-/usr/local/booster_robot/booster_robocup_sim}
EXT=$SIM/physics/extensions
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILTIN=$EXT/builtin.py

if [[ ! -f $BUILTIN ]]; then
  echo "not a Booster Studio virtual-robot container ($BUILTIN missing)"; exit 1
fi

HOST=$EXT/host.py
if [[ "${1:-install}" == "uninstall" ]]; then
  if [[ -f $BUILTIN.revo2.orig ]]; then mv -f "$BUILTIN.revo2.orig" "$BUILTIN"; fi
  if [[ -f $HOST.revo2.orig ]]; then mv -f "$HOST.revo2.orig" "$HOST"; fi
  rm -rf "$EXT/revo2_hand"
  echo "revo2_hand removed (builtin.py and host.py restored). Restart the container."
  exit 0
fi

mkdir -p "$EXT/revo2_hand"
cp "$SRC/revo2_hand/__init__.py" "$SRC/revo2_hand/extension.py" "$EXT/revo2_hand/"
[[ -f $BUILTIN.revo2.orig ]] || cp "$BUILTIN" "$BUILTIN.revo2.orig"
[[ -f $HOST.revo2.orig ]] || cp "$HOST" "$HOST.revo2.orig"

python3 - "$BUILTIN" <<'PY'
import sys, re
p = sys.argv[1]
s = open(p).read()
imp = "from physics.extensions.revo2_hand.extension import revo2_hand_physics_extension\n"
reg = "    register_physics_extension(revo2_hand_physics_extension)\n"
if imp not in s:
    s = s.replace("from physics.extensions.qpos_recorder.extension import qpos_recorder_physics_extension\n",
                  "from physics.extensions.qpos_recorder.extension import qpos_recorder_physics_extension\n" + imp)
if reg not in s:
    anchor = "    register_physics_extension(qpos_recorder_physics_extension)\n"
    if anchor not in s:
        sys.exit("builtin.py layout changed; patch it by hand (see README)")
    s = s.replace(anchor, anchor + reg)
open(p, "w").write(s)
print("patched", p)
PY

# host.py: activate revo2_hand for EVERY scene, even when the scene's extensions.json does not
# declare it (scenes saved by the Studio editor cannot declare physics extensions). The extension
# stays idle when the scene has no Revo2 actuators, so other scenes are unaffected.
python3 - "$HOST" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
begin = "        # --- robo@home revo2_hand: always on (idle without Revo2 actuators) ---\n"
if begin not in s:
    anchor = "\n    def initialize(self, model: Any, data: Any) -> None:\n"
    if anchor not in s:
        sys.exit("host.py layout changed; auto-activation not installed (declare revo2_hand in the scene instead)")
    block = (
        begin +
        "        if not any(c.name == 'revo2_hand' for c in self._contributions):\n"
        "            _revo2 = get_physics_extension('revo2_hand')\n"
        "            if _revo2 is not None:\n"
        "                _c = _PhysicsExtensionContribution(name='revo2_hand')\n"
        "                try:\n"
        "                    _revo2.contribute(PhysicsExtensionContext(_c, model_path=self._model_path,\n"
        "                                      simulation_config=self._simulation_config), _revo2.parse_config({}))\n"
        "                    self._commit_contribution(_c)\n"
        "                except Exception as exc:\n"
        "                    self._dispose_contribution(_c)\n"
        "                    print(f'[PhysicsExtensionHost] revo2_hand auto-activation failed: {exc}')\n"
        "        # --- end revo2_hand ---\n"
    )
    s = s.replace(anchor, "\n" + block + anchor, 1)
    open(p, "w").write(s)
    print("patched", p)
else:
    print("host.py already patched")
PY

# syntax check (importing the physics package itself needs the full ROS/sim environment)
python3 -m py_compile "$EXT/revo2_hand/extension.py" "$BUILTIN" "$HOST"
grep -q "register_physics_extension(revo2_hand_physics_extension)" "$BUILTIN" && echo "revo2_hand registered in builtin.py"
echo "Done. Now restart the container: docker restart <virtual-robot-container>"
