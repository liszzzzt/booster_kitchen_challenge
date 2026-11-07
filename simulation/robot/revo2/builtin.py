from __future__ import annotations

from physics.extensions.registry import get_physics_extension, register_physics_extension
from physics.extensions.qpos_recorder.extension import qpos_recorder_physics_extension
from physics.extensions.revo2_hand.extension import revo2_hand_physics_extension


_registered = False


def register_builtin_physics_extensions() -> None:
    global _registered
    if (
        _registered
        and get_physics_extension(qpos_recorder_physics_extension.name) is not None
    ):
        return
    _registered = True
    register_physics_extension(qpos_recorder_physics_extension)
    register_physics_extension(revo2_hand_physics_extension)
