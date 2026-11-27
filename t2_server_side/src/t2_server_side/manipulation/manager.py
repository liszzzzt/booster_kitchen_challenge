"""Server side of pick-and-place: object position -> manipulation_command sequence for the T2 agent.

    from t2_server_side.manipulation.manager import ManipulationManager
    mm = ManipulationManager(server_node)                  # T2ServerNode, already spinning
    mm.start()                                             # lock the legs, look down
    xyz = mm.locate("cup", radius=0.035)                   # vision (or give coordinates yourself)
    mm.pick("right", xyz, radius=0.035)
    mm.place("right", xyz + [0.0, 0.05, 0.0])              # 5 cm to the left
    mm.release()                                           # arms back, Booster takes the legs again

Coordinates are in the trunk frame (x forward, y left, z up), cylinder centre at grasp height. The robot
must not walk between start() and release() (the agent refuses locomotion meanwhile).
Robot side: t2_robot_side/src/robot/manipulation/pick_place.py (reach and motion details there).
"""
import itertools
import time

import numpy as np

COMMAND_TIMEOUT = 60.0
_ids = itertools.count(100000)


class ManipulationError(RuntimeError):
    pass


class ManipulationManager:
    def __init__(self, server_node, locator=None):
        self.node = server_node
        self.locator = locator          # vision.object_detection.locate.ObjectLocator, created on demand

    def _send(self, command, timeout=COMMAND_TIMEOUT):
        """Send and wait for SUCCESS / FAIL. Own wait loop: T2ServerNode.wait_for_success has a fixed
        15 s timeout, too short for a pick (the node must be spinning in another thread)."""
        command = {"id": next(_ids), **command}
        self.node.send_manipulation_command(command)
        deadline = time.monotonic() + timeout
        while True:
            state = self.node.command_status.get(command["id"])
            if state == "SUCCESS":
                return
            if state == "FAIL" or time.monotonic() > deadline:
                raise ManipulationError(f"{command['type']} failed ({state or 'no answer'}); see the T2 agent log")
            time.sleep(0.05)

    def start(self, head_pitch=0.35, leg_gain_scale=3.0):
        self._send({"type": "manip_start", "head_pitch": head_pitch, "leg_gain_scale": leg_gain_scale})

    def pick(self, side, coordinate, radius=0.035, pitch=0.0, roll=0.0, shapes=None):
        """pitch / roll [deg]: hand turned (see pick_place.py), e.g. roll 25 = palm facing down-inwards.
        shapes: hand shapes to try in turn until one holds (pick_place.HAND_SHAPES), default ["power"]."""
        command = {"type": "pick", "side": side, "coordinate": [float(v) for v in coordinate], "radius": radius,
                   "pitch": float(pitch), "roll": float(roll)}
        if shapes:
            command["shapes"] = list(shapes)
        self._send(command, timeout=COMMAND_TIMEOUT + 30.0 * len(shapes or []))

    def place(self, side, coordinate):
        self._send({"type": "place", "side": side, "coordinate": [float(v) for v in coordinate]})

    def release(self):
        self._send({"type": "manip_release"})

    def locate(self, label, radius=0.035, tries=5):
        """Cylinder centre of `label` in the trunk frame, median of a few detections."""
        import rclpy
        from t2_server_side.vision.object_detection.locate import LocateError, ObjectLocator, wait_ready
        if self.locator is None:
            self.locator = ObjectLocator()
        problem = wait_ready(self.locator)
        if problem:
            raise ManipulationError(f"vision not ready: {problem}")
        points, last_error = [], None
        for _ in range(tries):
            for _ in range(5):
                rclpy.spin_once(self.locator, timeout_sec=0.05)
            try:
                points.append(self.locator.locate(label, radius)[0])
            except LocateError as error:
                last_error = error
        if not points:
            raise ManipulationError(f"cannot locate {label}: {last_error}")
        return np.median(points, axis=0)
